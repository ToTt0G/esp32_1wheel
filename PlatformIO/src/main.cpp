#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Preferences.h>
#include <esp_task_wdt.h>          // Fix #11: Watchdog timer
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "ble_server.h"

#define VELOSTAT_PIN 32
#define MPU_SCL 22
#define MPU_SDA 23

// Hoverboard UART Config
#define HOVER_SERIAL_BAUD   115200
#define START_FRAME         0xABCD
#define TIME_SEND           10000  // Fix #9: 10ms in microseconds for micros() scheduling
#define TIME_CONTROL_LOOP   5000   // Fix #9: 5ms in microseconds for micros() scheduling
#define HOVER_RX_PIN        16
#define HOVER_TX_PIN        17

// Wheel speed conversion
#define WHEEL_DIAMETER_MM   240.0f
#define MOTOR_POLE_PAIRS    15
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER_MM * PI)

Adafruit_MPU6050 mpu;

// Ride parameters
float targetAngle = 0.0;
bool  isArmed     = false;

// Pushback Config
const float PUSHBACK_MAX_SPEED_KMH   = 20.0;
const float PUSHBACK_START_SPEED_KMH = 15.0;
const float PUSHBACK_MAX_ANGLE       = 4.0;

// Dirty Landing Config
const unsigned long FAULT_TIMEOUT_MS  = 500;
const float         FAULT_MIN_SPEED_KMH = 2.0;
unsigned long footpadTimerStart = 0;

// PID Configuration — Fix #6: Default footpad threshold raised to 1500
float Kp = 30.0;
float Ki = 0.5;
float Kd = 0.5;
float footpadThreshold = 1500.0;  // Fix #6: was 10 — far too low for ADC noise floor
float integral    = 0.0;
float prevError   = 0.0;
float pitchOffset = 0.0f;
volatile bool calibratePending = false;

// PT1 Filter state for Kd
float prevDerivative = 0.0;
const float DERIVATIVE_ALPHA = 0.3;

// Fix #1/#2: Thread-safe pending buffers — ALL written under g_stateMutex
SemaphoreHandle_t g_stateMutex = nullptr;

volatile bool  pidUpdatePending = false;
volatile float pendingKp, pendingKi, pendingKd, pendingFootpadThreshold;

volatile bool armedUpdatePending = false;
volatile bool pendingIsArmed     = false;

volatile bool rebootPending    = false;  // Fix #3
volatile bool nvsPidDirty      = false;  // Fix #4: deferred NVS write flag

// Mahony filter variables
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
const float twoKpDef = (2.0f * 0.4f);
const float twoKiDef = (2.0f * 0.0f);

// Hoverboard Protocol Structs
#pragma pack(push, 1)
typedef struct {
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;

typedef struct {
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
#pragma pack(pop)

SerialCommand  Command;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

uint8_t idx = 0;
uint16_t bufStartFrame;
byte *p;
byte incomingByte;
byte incomingBytePrev;

// Fix #10: UART frame receive timeout
unsigned long lastByteTime = 0;
const unsigned long FRAME_TIMEOUT_MS = 20;

// Global state
float    currentPitch = 0;
float    currentRoll  = 0;
int16_t  uSpeed = 0;
int16_t  uSteer = 0;
float    speedKmh    = 0;
uint16_t footpadAdc  = 4095;
bool footpadPressed  = false;
bool footpadLeft     = false;
bool footpadRight    = false;

// Function Prototypes
void Send(int16_t uSteer, int16_t uSpeed);
void Receive();
float calculatePID(float currentAngle, float target, float dt);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);

// Fix #8: Safe invSqrt using memcpy — avoids strict-aliasing UB
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    int32_t i;
    memcpy(&i, &y, sizeof(i));          // type-pun safely
    i = 0x5f3759df - (i >> 1);
    memcpy(&y, &i, sizeof(y));
    y = y * (1.5f - (halfx * y * y));  // one Newton-Raphson step
    return y;
}

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Onewheel Initializing...");

    // Fix #1: Create mutex before ble_init() which may be called from another task context
    g_stateMutex = xSemaphoreCreateMutex();
    configASSERT(g_stateMutex);

    // Currently using the internal pull-up (~45kΩ) so no extra components are needed.
    // This works, but the ESP32 ADC is non-linear near the rails — with no foot pressure
    // the pin idles at ~4095, which is the least accurate region of the ADC.
    //
    // UPGRADE: For better accuracy and cross-board consistency, add a 10kΩ resistor:
    //   - Use a standard 10kΩ 1/4W through-hole resistor (e.g., 10k 1% metal film)
    //   - Wire it between GPIO32 and 3.3V (in parallel with the internal pull-up)
    //   - This pulls the idle voltage lower (~2.8–3.0V / ADC ~3400) — a more linear zone
    //   - Change the line below to: pinMode(VELOSTAT_PIN, INPUT);
    //   - The velostat goes between GPIO32 and GND (same as now)
    //   - Schematic: 3.3V ---[10kΩ]--- GPIO32 ---[Velostat]--- GND
    pinMode(VELOSTAT_PIN, INPUT_PULLUP);

    Wire.begin(MPU_SDA, MPU_SCL);
    Wire.setTimeOut(10);
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip - Check wiring!");
        while (1) delay(10);
    }
    Serial.println("MPU6050 Found!");

    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);

    Serial2.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, HOVER_RX_PIN, HOVER_TX_PIN);
    pinMode(LED_BUILTIN, OUTPUT);

    ble_init();

    // Fix #9: Seed micros()-based timers
    // (done inside loop via static init — nothing needed here)

    // Sanity-check NVS-loaded footpad threshold
    if (footpadThreshold < 100.0 || footpadThreshold > 4095.0) {
        Serial.printf("[WARN] Footpad threshold %.1f out of range, resetting to 1500\n", footpadThreshold);
        footpadThreshold = 1500.0;
    }

    // Fix #11: Watchdog — panic if loop() stalls for more than 2 seconds
    esp_task_wdt_init(2, true);
    esp_task_wdt_add(NULL);

    Serial.println("[SYSTEM] Warmup starting (3 seconds)...");
}

void loop() {
    // Fix #11: Feed watchdog each iteration
    esp_task_wdt_reset();

    unsigned long timeNow      = millis();
    unsigned long timeNowMicros = micros();

    // Fix #9: All scheduling now uses micros() for sub-ms precision
    static unsigned long iTimeSendUs    = 0;
    static unsigned long iTimeControlUs = 0;
    static bool warmupComplete = false;

    // Process incoming hoverboard telemetry (run frequently)
    Receive();

    // ── Safety Startup Delay ─────────────────────────────────────────
    if (!warmupComplete) {
        digitalWrite(LED_BUILTIN, (timeNow / 200) % 2);

        static unsigned long lastWarmupLog = 0;
        if (timeNow - lastWarmupLog > 500) {
            lastWarmupLog = timeNow;
            Serial.printf("[WARMUP] t:%lu P:%.1fdeg Foot:%d Thr:%.0f\n",
                          timeNow, currentPitch, analogRead(VELOSTAT_PIN), footpadThreshold);
        }

        if (timeNow > 3000) {
            warmupComplete = true;
            isArmed        = true;
            // Fix #9: Seed micros timers at warmup completion to avoid t=0 burst
            iTimeControlUs = timeNowMicros;
            iTimeSendUs    = timeNowMicros;
            digitalWrite(LED_BUILTIN, HIGH);
            Serial.printf("[SYSTEM] Warmup done. Board ARMED. FP threshold=%.0f\n", footpadThreshold);
        }

        Send(0, 0);
        return;
    }

    // Fix #3: Handle pending reboot — stop motor first, then restart
    if (rebootPending) {
        Send(0, 0);
        delay(200);
        Serial.println("[CMD] REBOOT executing now.");
        ESP.restart();
    }

    // ── Primary High-Speed Control Loop (200Hz via micros) ───────────
    if ((long)(timeNowMicros - iTimeControlUs) >= 0) {
        iTimeControlUs = timeNowMicros + TIME_CONTROL_LOOP;

        static unsigned long prevMicros = 0;
        float dt = (timeNowMicros - prevMicros) / 1000000.0f;
        if (dt <= 0 || dt > 0.1f) dt = 0.005f;  // Fallback to 5ms
        prevMicros = timeNowMicros;

        // 1. Read Velostat Footpad (Debounced)
        static bool stableFootpad = false;
        static unsigned long footpadChangeTime = 0;
        footpadAdc = analogRead(VELOSTAT_PIN);

        // Fix #6: Footpad is pressed when ADC drops below threshold
        // (velostat resistance decreases under pressure → lower ADC reading)
        bool rawFootpad = (footpadAdc < (uint16_t)footpadThreshold);

        if (rawFootpad != stableFootpad) {
            if (footpadChangeTime == 0) footpadChangeTime = timeNow;
            if (timeNow - footpadChangeTime > 50) {
                stableFootpad      = rawFootpad;
                footpadChangeTime  = 0;
            }
        } else {
            footpadChangeTime = 0;
        }

        footpadLeft    = stableFootpad;
        footpadRight   = stableFootpad;
        footpadPressed = footpadLeft && footpadRight;

        // 2. Read MPU6050 & Run Mahony Filter
        sensors_event_t a, g, temp;
        if (mpu.getEvent(&a, &g, &temp)) {
            MahonyAHRSupdateIMU(g.gyro.x, g.gyro.y, g.gyro.z,
                                a.acceleration.x, a.acceleration.y, a.acceleration.z, dt);
        }

        // Convert Quaternions to Euler Angles
        float rawPitch  = atan2(2.0f * (q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3) * 180.0f / PI;
        currentPitch    = rawPitch - pitchOffset;
        currentRoll     = -asin(2.0f * (q0*q2 - q3*q1)) * 180.0f / PI;
        // Fix #14: yaw removed — computed but never used

        // Speed — use raw electrical RPM → km/h
        float mechRPM = (float)abs(Feedback.speedL_meas) / MOTOR_POLE_PAIRS;
        speedKmh      = (mechRPM * WHEEL_CIRCUMFERENCE * 60.0f) / 1000000.0f;

        // Fix #7: Direction from motor feedback sign only — not from pitch
        if (Feedback.speedL_meas < 0) {
            speedKmh = -speedKmh;
        }

        // 3. Board Safety Limits
        bool isFallen      = (fabs(currentPitch) > 40.0) || (fabs(currentRoll) > 40.0);
        bool isLevelForStart = (fabs(currentRoll) < 15.0) && (fabs(currentPitch) < 5.0);

        // 4. Balancing & Fault Delay Logic
        static bool isBalancing = false;

        if (!isBalancing) {
            if (footpadPressed && isArmed && !isFallen && isLevelForStart) {
                isBalancing      = true;
                footpadTimerStart = 0;
            }
        } else {
            if (isFallen || !isArmed) {
                isBalancing       = false;
                footpadTimerStart = 0;
            } else if (!footpadPressed) {
                if (fabs(speedKmh) > FAULT_MIN_SPEED_KMH) {
                    if (footpadTimerStart == 0) {
                        footpadTimerStart = timeNow;
                    } else if (timeNow - footpadTimerStart > FAULT_TIMEOUT_MS) {
                        isBalancing       = false;
                        footpadTimerStart = 0;
                    }
                } else {
                    isBalancing       = false;
                    footpadTimerStart = 0;
                }
            } else {
                footpadTimerStart = 0;
            }
        }

        // 5. Pushback Logic
        if (isBalancing) {
            if (fabs(speedKmh) >= PUSHBACK_START_SPEED_KMH) {
                float surplusSpeed  = fabs(speedKmh) - PUSHBACK_START_SPEED_KMH;
                float pushbackRatio = surplusSpeed / (PUSHBACK_MAX_SPEED_KMH - PUSHBACK_START_SPEED_KMH);
                pushbackRatio       = constrain(pushbackRatio, 0.0, 1.0);
                targetAngle = (speedKmh > 0)
                    ?  (pushbackRatio * PUSHBACK_MAX_ANGLE)
                    : -(pushbackRatio * PUSHBACK_MAX_ANGLE);
            } else {
                targetAngle = 0.0;
            }
        } else {
            targetAngle = 0.0;
        }

        // 6. Compute Control Action
        if (isBalancing) {
            float pidOutput = calculatePID(currentPitch, targetAngle, dt);
            uSpeed = (int16_t)constrain(-pidOutput, -1000, 1000);
        } else {
            uSpeed    = 0;
            integral  = 0;
            prevError = targetAngle - currentPitch;
        }
    }

    // ── Slower UART Transmission & BLE Loop (100Hz via micros) ───────
    if ((long)(timeNowMicros - iTimeSendUs) >= 0) {
        iTimeSendUs = timeNowMicros + TIME_SEND;

        // Fix #1/#2: Consume all pending updates under mutex
        xSemaphoreTake(g_stateMutex, portMAX_DELAY);

        if (pidUpdatePending) {
            float oldThr     = footpadThreshold;
            Kp               = pendingKp;
            Ki               = pendingKi;
            Kd               = pendingKd;
            footpadThreshold = pendingFootpadThreshold;
            prevError        = targetAngle - currentPitch;
            integral         = 0;
            pidUpdatePending = false;
            Serial.printf("[PID] Applied: Kp=%.2f Ki=%.2f Kd=%.2f Thr=%.1f\n", Kp, Ki, Kd, footpadThreshold);

            // Fix #4: Mark NVS dirty only if threshold changed; actual write is deferred below
            if (fabs(footpadThreshold - oldThr) > 0.01f) {
                nvsPidDirty = true;
            }
        }

        if (armedUpdatePending) {
            // Fix #2: Apply arm/disarm only when it's safe (handled here in main task)
            isArmed             = pendingIsArmed;
            armedUpdatePending  = false;
            Serial.printf("[ARM] Applied: %s\n", isArmed ? "ARMED" : "DISARMED");
        }

        xSemaphoreGive(g_stateMutex);

        // Handle tilt sensor calibration
        if (calibratePending) {
            pitchOffset      = currentPitch + pitchOffset;
            calibratePending = false;
            Serial.printf("[CAL] New pitch offset: %.2f\n", pitchOffset);
            nvsPidDirty = true;  // Reuse dirty flag to persist offset too
        }

        // Fix #4: Deferred NVS write — only runs in the 100Hz send slot,
        // never inside the 200Hz control loop, and only when a value changed.
        if (nvsPidDirty) {
            nvsPidDirty = false;
            Preferences prefs;
            prefs.begin("onewheel", false);
            prefs.putFloat("fpThr", footpadThreshold);
            prefs.putFloat("pOff",  pitchOffset);
            prefs.end();
            Serial.printf("[NVS] Auto-saved: Thr=%.1f Off=%.2f\n", footpadThreshold, pitchOffset);
        }

        // 1. Send to Hoverboard
        Send(uSteer, uSpeed);

        // 2. BLE Telemetry (~10Hz)
        static int bleCounter = 0;
        if (bleCounter++ >= 10) {
            bleCounter = 0;

            float batVolts = Feedback.batVoltage / 100.0f;

            uint16_t flags = 0;
            if (footpadLeft)                              flags |= FLAG_FOOTPAD_LEFT;
            if (footpadRight)                             flags |= FLAG_FOOTPAD_RIGHT;
            if (isArmed)                                  flags |= FLAG_ARMED;
            if (Feedback.boardTemp > 600)                 flags |= FLAG_OVER_TEMP;
            if (batVolts > 1.0f && batVolts < 50.0f)     flags |= FLAG_LOW_BATTERY;

            TelemetryPacket pkt;
            pkt.batteryVoltage  = batVolts;
            pkt.speed           = speedKmh;
            pkt.pitch           = currentPitch;
            pkt.boardTemp       = Feedback.boardTemp;
            pkt.motorCurrentL   = Feedback.cmd1;
            pkt.motorCurrentR   = Feedback.cmd2;
            pkt.statusFlags     = flags;
            pkt.footpadAdc      = footpadAdc;
            pkt.footpadThreshold = (int16_t)footpadThreshold;

            ble_sendTelemetry(pkt);
        }

        // 3. Debug output (~2Hz)
        static int debugCounter = 0;
        if (debugCounter++ >= 50) {
            debugCounter = 0;
            float batVoltsDbg = Feedback.batVoltage / 100.0f;
            float error       = targetAngle - currentPitch;

            Serial.printf("[RIDE] V:%.1fV P:%.1fdeg R:%.1fdeg Spd:%.1f Tgt:%.1fdeg F:%s%s Out:%d\n",
                batVoltsDbg, currentPitch, currentRoll, speedKmh, targetAngle,
                footpadLeft ? "L" : "-", footpadRight ? "R" : "-", uSpeed);

            Serial.printf("[DIAG] Err:%.2f I:%.2f dI:%.2f BLE:%s faultTimer:%lu\n",
                error, integral, prevDerivative,
                ble_isConnected() ? "ON" : "OFF",
                footpadTimerStart == 0 ? 0UL : (timeNow - footpadTimerStart));
        }
    }
}

float calculatePID(float currentAngle, float target, float dt) {
    float error  = target - currentAngle;
    integral    += error * dt;
    integral     = constrain(integral, -25.0, 25.0);  // Anti-windup

    float rawDerivative      = (error - prevError) / dt;
    prevError                = error;

    float filteredDerivative = (DERIVATIVE_ALPHA * rawDerivative) + ((1.0 - DERIVATIVE_ALPHA) * prevDerivative);
    prevDerivative           = filteredDerivative;

    return (Kp * error) + (Ki * integral) + (Kd * filteredDerivative);
}

// Mahony AHRS algorithm implementation
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = invSqrt(ax*ax + ay*ay + az*az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        halfvx = q1*q3 - q0*q2;
        halfvy = q0*q1 + q2*q3;
        halfvz = q0*q0 - 0.5f + q3*q3;

        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        if (twoKiDef > 0.0f) {
            integralFBx += twoKiDef * halfex * dt;
            integralFBy += twoKiDef * halfey * dt;
            integralFBz += twoKiDef * halfez * dt;
            gx += integralFBx; gy += integralFBy; gz += integralFBz;
        } else {
            integralFBx = integralFBy = integralFBz = 0.0f;
        }

        gx += twoKpDef * halfex;
        gy += twoKpDef * halfey;
        gz += twoKpDef * halfez;
    }

    gx *= (0.5f * dt); gy *= (0.5f * dt); gz *= (0.5f * dt);
    qa = q0; qb = q1; qc = q2;
    q0 += (-qb*gx - qc*gy - q3*gz);
    q1 += ( qa*gx + qc*gz - q3*gy);
    q2 += ( qa*gy - qb*gz + q3*gx);
    q3 += ( qa*gz + qb*gy - qc*gx);

    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

void Send(int16_t uSteer, int16_t uSpeed) {
    Command.start    = (uint16_t)START_FRAME;
    Command.steer    = uSteer;
    Command.speed    = uSpeed;
    Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);
    Serial2.write((uint8_t*)&Command, sizeof(Command));
}

void Receive() {
    static unsigned long lastRxDiag  = 0;
    static uint32_t rawBytesTotal    = 0;
    static uint32_t goodFrames       = 0;
    static uint32_t badChecksum      = 0;

    // Fix #10: Stale partial-frame timeout — reset parser if no byte arrives
    // within FRAME_TIMEOUT_MS while mid-frame
    unsigned long now = millis();
    if (idx > 0 && (now - lastByteTime) > FRAME_TIMEOUT_MS) {
        Serial.printf("[RX] Frame timeout — resetting parser (idx was %d)\n", idx);
        idx = 0;
    }

    while (Serial2.available()) {
        incomingByte = Serial2.read();
        lastByteTime = millis();
        rawBytesTotal++;
        bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;

        if (idx == 0 && bufStartFrame == START_FRAME) {
            p      = (byte*)&NewFeedback;
            *p++   = incomingBytePrev;
            *p++   = incomingByte;
            idx    = 2;
        } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {
            *p++ = incomingByte;
            idx++;
        }

        if (idx == sizeof(SerialFeedback)) {
            uint16_t checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2
                                ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                                ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

            if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
                memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
                goodFrames++;
            } else {
                badChecksum++;
            }
            idx = 0;
        }

        incomingBytePrev = incomingByte;
    }

    if (millis() - lastRxDiag > 5000) {
        lastRxDiag = millis();
        Serial.printf("[RX] bytes:%lu goodFrames:%lu badCRC:%lu batRaw:%d (%.2fV)\n",
            rawBytesTotal, goodFrames, badChecksum,
            Feedback.batVoltage, Feedback.batVoltage / 100.0f);
    }
}