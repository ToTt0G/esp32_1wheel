#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "ble_server.h"

#define VELOSTAT_PIN 32 // Pin 21 does NOT support ADC. Use 32 or 33 instead.
#define MPU_SCL 22
#define MPU_SDA 23

// Hoverboard UART Config
#define HOVER_SERIAL_BAUD   115200 
#define START_FRAME         0xABCD
#define TIME_SEND           10 // 10ms (100Hz) to hoverboard
#define TIME_CONTROL_LOOP   5  // 5ms (200Hz) balancing loop
#define HOVER_RX_PIN        16
#define HOVER_TX_PIN        17

// Wheel speed conversion
#define WHEEL_DIAMETER_MM   240.0f    // 24cm wheel
#define MOTOR_POLE_PAIRS    15        // standard hoverboard BLDC hub motor
#define WHEEL_CIRCUMFERENCE (WHEEL_DIAMETER_MM * PI)  // ~753.98mm

Adafruit_MPU6050 mpu;

// Ride parameters
float targetAngle = 0.0; // Level is 0 degrees
bool  isArmed     = true; // BLE kill switch — starts ARMED by default

// Pushback Config
const float PUSHBACK_MAX_SPEED_KMH = 20.0;      // Speed at which max pushback occurs
const float PUSHBACK_START_SPEED_KMH = 15.0;    // Speed at which pushback starts
const float PUSHBACK_MAX_ANGLE = 4.0;           // Maximum nose-up angle in degrees

// Dirty Landing Config
const unsigned long FAULT_TIMEOUT_MS = 500;
const float FAULT_MIN_SPEED_KMH = 2.0;
unsigned long footpadTimerStart = 0;

// PID Configuration
float Kp = 60.0;
float Ki = 0.5;
float Kd = 2.0;
float footpadThreshold = 2000.0;
float integral = 0.0;
float prevError = 0.0;
unsigned long prevTime_PID = 0;

// PT1 Filter state for Kd
float prevDerivative = 0.0;
const float DERIVATIVE_ALPHA = 0.3; // Low-pass filter weight (0.0 to 1.0, lower is smoother)

// Thread-safe PID update buffer (written by BLE task, consumed by loop)
volatile bool  pidUpdatePending = false;
volatile float pendingKp, pendingKi, pendingKd, pendingFootpadThreshold;

// Mahony filter variables
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; 
float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
const float twoKpDef = (2.0f * 0.4f);  // 2 * proportional gain
const float twoKiDef = (2.0f * 0.0f);  // 2 * integral gain

// Hoverboard Protocol Structs
#pragma pack(push, 1)
typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;

typedef struct{
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

SerialCommand Command;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

uint8_t idx = 0;
uint16_t bufStartFrame;
byte *p;
byte incomingByte;
byte incomingBytePrev;

// Global state variables for loop processing
float currentPitch = 0;
float currentRoll = 0;
int16_t uSpeed = 0;
int16_t uSteer = 0;
float speedKmh = 0;
uint16_t footpadAdc = 4095;
bool footpadPressed = false;
bool footpadLeft = false;
bool footpadRight = false;


// Function Prototypes
void Send(int16_t uSteer, int16_t uSpeed);
void Receive();
float calculatePID(float currentAngle, float target, float dt);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);

void setup() {
  Serial.begin(115200);

  Serial.println("ESP32 Onewheel Initializing...");

  pinMode(VELOSTAT_PIN, INPUT_PULLUP);

  Wire.begin(MPU_SDA, MPU_SCL); 
  Wire.setTimeOut(10); // 10ms I2C timeout to prevent bus lockups
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip - Check wiring!");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ); // Increased bandwidth for 500Hz loop

  // Initialize Serial2 for Hoverboard Comms
  Serial2.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, HOVER_RX_PIN, HOVER_TX_PIN);

  // Setup basic LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize BLE GATT server (must be after Serial)
  ble_init();

  // Initialize PID timing to avoid garbage dt on first cycle
  prevTime_PID = millis();
}

void loop() {
  unsigned long timeNow = millis();
  unsigned long timeNowMicros = micros();
  static unsigned long iTimeSend = 0;
  static unsigned long iTimeControl = 0;
  
  // Process incoming hoverboard telemetry (run frequently)
  Receive();

  // Primary High-Speed Control Loop (500Hz)
  if ((long)(timeNow - iTimeControl) >= 0) {
      iTimeControl = timeNow + TIME_CONTROL_LOOP;
      
      // Time delta for integrations
      static unsigned long prevMicros = 0;
      float dt = (timeNowMicros - prevMicros) / 1000000.0f;
      if (dt <= 0 || dt > 0.1f) dt = 0.002f; // Fallback to 2ms
      prevMicros = timeNowMicros;

      // 1. Read Velostat Footpad (Debounced & Virtual Dual-Zone)
      static bool stableFootpad = false;
      static unsigned long footpadChangeTime = 0;
      footpadAdc = analogRead(VELOSTAT_PIN);
      bool rawFootpad = (footpadAdc < footpadThreshold);
      
      // Debounce logic (50ms)
      if (rawFootpad != stableFootpad) {
          if (footpadChangeTime == 0) footpadChangeTime = timeNow;
          if (timeNow - footpadChangeTime > 50) {
              stableFootpad = rawFootpad;
              footpadChangeTime = 0;
          }
      } else {
          footpadChangeTime = 0;
      }
      
      // Single physical sensor mapped to both zones for now
      footpadLeft = stableFootpad;
      footpadRight = stableFootpad;
      footpadPressed = footpadLeft && footpadRight;

      // 2. Read MPU6050 & Run Mahony Filter
      sensors_event_t a, g, temp;
      if (mpu.getEvent(&a, &g, &temp)) {
          // MPU6050 readings in standard format for Mahony (rad/s and m/s^2)
          MahonyAHRSupdateIMU(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x, a.acceleration.y, a.acceleration.z, dt);
      }

      // Convert Quaternions to Euler Angles
      // Physical Board Pitch is rotation around the MPU's X axis
      currentPitch = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 180.0f / PI;
      // Physical Board Roll is rotation around the MPU's Y axis
      currentRoll  = -asin(2.0f * (q0 * q2 - q3 * q1)) * 180.0f / PI;
      float yaw    = atan2(2.0f * (q0 * q3 + q1 * q2), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 180.0f / PI;

      // Convert raw electrical RPM to km/h map to absolute speed
      float mechRPM = (float)abs(Feedback.speedL_meas) / MOTOR_POLE_PAIRS;
      speedKmh = (mechRPM * WHEEL_CIRCUMFERENCE * 60.0f) / 1000000.0f;
      
      // Speed Direction Mapping
      if (currentPitch > 0 && Feedback.speedL_meas < 0) { // Moving backwards relative to nose
          speedKmh = -speedKmh;
      } else if (currentPitch < 0 && Feedback.speedL_meas > 0) {
          speedKmh = -speedKmh;
      }


      // 3. Board Safety Limits (Fallen & Roll Tolerance)
      bool isFallen = (fabs(currentPitch) > 40.0) || (fabs(currentRoll) > 40.0);
      bool isLevelForStart = (fabs(currentRoll) < 15.0) && (fabs(currentPitch) < 15.0); // Must be relatively level to start
      
      // 4. Fault Delay Logic (Dirty Landings)
      static bool isBalancing = false;
      
      if (footpadPressed && !isFallen && isArmed && isLevelForStart) {
          isBalancing = true;
          footpadTimerStart = 0;
      } else if (isBalancing) {
          if (isFallen || !isArmed) {
             // Hard kill (flipped over or disarmed)
             isBalancing = false; 
          } else if (!footpadPressed) {
             // Footpad lost - check speed for dirty landing delay
             if (fabs(speedKmh) > FAULT_MIN_SPEED_KMH) {
                 if (footpadTimerStart == 0) {
                     footpadTimerStart = timeNow;
                 } else if (timeNow - footpadTimerStart > FAULT_TIMEOUT_MS) {
                     // Timer expired, disengage
                     isBalancing = false;
                     footpadTimerStart = 0;
                 }
                 // If timer hasn't expired, isBalancing remains true!
             } else {
                 // Slow speed dismount (heel-lift or jumping off)
                 isBalancing = false;
             }
          }
      } else {
          footpadTimerStart = 0;
      }


      // 5. Pushback Logic
      if (isBalancing) {
          if (fabs(speedKmh) >= PUSHBACK_START_SPEED_KMH) {
              float surplusSpeed = fabs(speedKmh) - PUSHBACK_START_SPEED_KMH;
              float pushbackRatio = surplusSpeed / (PUSHBACK_MAX_SPEED_KMH - PUSHBACK_START_SPEED_KMH);
              pushbackRatio = constrain(pushbackRatio, 0.0, 1.0);
              
              // Lift nose (positive angle points nose up)
              // If going backwards (negative speed), we point nose DOWN to push tail UP
              if (speedKmh > 0) {
                  targetAngle = pushbackRatio * PUSHBACK_MAX_ANGLE;
              } else {
                  targetAngle = -1.0f * (pushbackRatio * PUSHBACK_MAX_ANGLE);
              }
          } else {
              targetAngle = 0.0;
          }
      } else {
          targetAngle = 0.0;
      }

      // 6. Compute Control Action
      if (isBalancing) {
        float pidOutput = calculatePID(currentPitch, targetAngle, dt);
        uSpeed = (int16_t)constrain(pidOutput, -1000, 1000);
      } else {
        // E-Stop / Disengage
        uSpeed = 0;
        integral = 0;
        prevError = targetAngle - currentPitch;  // prevent derivative spike on re-engage
      }
  }


  // Slower UART Transmission & BLE Loop (100Hz)
  if ((long)(timeNow - iTimeSend) >= 0) {
    iTimeSend = timeNow + TIME_SEND;

    // Apply pending PID update from BLE
    if (pidUpdatePending) {
        Kp = pendingKp;
        Ki = pendingKi;
        Kd = pendingKd;
        footpadThreshold = pendingFootpadThreshold;
        prevError = targetAngle - currentPitch;
        integral = 0;
        pidUpdatePending = false;
        Serial.printf("[PID] Applied: Kp=%.2f Ki=%.2f Kd=%.2f Thr=%.0f\n", Kp, Ki, Kd, footpadThreshold);
    }

    // 1. Send to Hoverboard Motherboard
    Send(uSteer, uSpeed);

    // 2. Build & send BLE telemetry packet (~10Hz)
    static int bleCounter = 0;
    if (bleCounter++ >= 10) {
      bleCounter = 0;

      float batVolts = Feedback.batVoltage / 100.0f;

      // Build status flags bitmask
      uint16_t flags = 0;
      if (footpadLeft)  flags |= FLAG_FOOTPAD_LEFT;
      if (footpadRight) flags |= FLAG_FOOTPAD_RIGHT;
      if (isArmed)        flags |= FLAG_ARMED;
      if (Feedback.boardTemp > 600)  flags |= FLAG_OVER_TEMP;
      if (batVolts > 1.0f && batVolts < 50.0f) flags |= FLAG_LOW_BATTERY;

      TelemetryPacket pkt;
      pkt.batteryVoltage = batVolts;
      pkt.speed          = speedKmh; // Absolute speed calculation moved to 500Hz loop
      pkt.pitch          = currentPitch;
      pkt.boardTemp      = Feedback.boardTemp;
      pkt.motorCurrentL  = Feedback.cmd1;
      pkt.motorCurrentR  = Feedback.cmd2;
      pkt.statusFlags    = flags;
      pkt.footpadAdc     = footpadAdc;
      pkt.footpadThreshold = (int16_t)footpadThreshold;

      ble_sendTelemetry(pkt);
    }

    // 3. Debug output (~2Hz)
    static int debugCounter = 0;
    if (debugCounter++ >= 50) {
      debugCounter = 0;
      float batVoltsDbg = Feedback.batVoltage / 100.0f;
      float error = targetAngle - currentPitch;

      // Line 1: Core riding state
      Serial.printf("[RIDE] V:%.1fV P:%.1f° R:%.1f° Spd:%.1f Tgt:%.1f° F:%s%s Out:%d\n",
        batVoltsDbg, currentPitch, currentRoll, speedKmh, targetAngle,
        footpadLeft ? "L" : "-", footpadRight ? "R" : "-",
        uSpeed);

      // Line 2: Diagnostics
      Serial.printf("[DIAG] Err:%.2f I:%.2f dI:%.2f BLE:%s faultTimer:%lu\n",
        error, integral, prevDerivative,
        ble_isConnected() ? "ON" : "OFF",
        footpadTimerStart == 0 ? 0 : (timeNow - footpadTimerStart));
    }
  }

}

float calculatePID(float currentAngle, float target, float dt) {
  float error = target - currentAngle;
  integral += error * dt;

  // Anti-windup cap
  integral = constrain(integral, -25.0, 25.0);

  float rawDerivative = (error - prevError) / dt;
  prevError = error;

  // PT1 Low-pass filter for derivative
  float filteredDerivative = (DERIVATIVE_ALPHA * rawDerivative) + ((1.0 - DERIVATIVE_ALPHA) * prevDerivative);
  prevDerivative = filteredDerivative;

  float output = (Kp * error) + (Ki * integral) + (Kd * filteredDerivative);
  return output;
}

// ---------------------------------------------------------------------------------------------------
// Fast inverse square-root
float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

// Mahony AHRS algorithm implementation
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if(twoKiDef > 0.0f) {
            integralFBx += twoKiDef * halfex * dt;
            integralFBy += twoKiDef * halfey * dt;
            integralFBz += twoKiDef * halfez * dt;
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKpDef * halfex;
        gy += twoKpDef * halfey;
        gz += twoKpDef * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

void Send(int16_t uSteer, int16_t uSpeed) {
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Send the entire 8-byte frame in one burst. Hardware FIFO prevents mid-frame gaps.
  Serial2.write((uint8_t *) &Command, sizeof(Command)); 
}

void Receive() {
    while (Serial2.available()) {
        incomingByte = Serial2.read();
        bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;

        if (idx == 0 && bufStartFrame == START_FRAME) {
            p       = (byte *)&NewFeedback;
            *p++    = incomingBytePrev;
            *p++    = incomingByte;
            idx     = 2;
        } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {
            *p++    = incomingByte;
            idx++;
        }

        if (idx == sizeof(SerialFeedback)) {
            uint16_t checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                                ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

            if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
                memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
            }
            idx = 0;
        }

        incomingBytePrev = incomingByte;
    }
}