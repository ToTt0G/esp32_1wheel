#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "ble_server.h"

#define VELOSTAT_PIN 21
#define MPU_SCL 22
#define MPU_SDA 23

// Hoverboard UART Config
#define HOVER_SERIAL_BAUD   115200 
#define START_FRAME         0xABCD 
#define TIME_SEND           20    // 20ms control loop for balancing
#define HOVER_RX_PIN        16
#define HOVER_TX_PIN        17

Adafruit_MPU6050 mpu;

// PID Configuration
// Note: These values will need tuning. Kp acts as main stiffness, Kd dampens oscillations.
float Kp = 60.0;
float Ki = 0.5;
float Kd = 2.0;
float targetAngle = 0.0; // Level is 0 degrees
bool  isArmed     = false; // BLE kill switch — starts DISARMED for safety

float integral = 0.0;
float prevError = 0.0;
unsigned long prevTime_PID = 0;

// Thread-safe PID update buffer (written by BLE task, consumed by loop)
volatile bool  pidUpdatePending = false;
volatile float pendingKp, pendingKi, pendingKd;

// Hoverboard Protocol Structs
typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

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
SerialFeedback Feedback;
SerialFeedback NewFeedback;

uint8_t idx = 0;
uint16_t bufStartFrame;
byte *p;
byte incomingByte;
byte incomingBytePrev;

// Function Prototypes
void Send(int16_t uSteer, int16_t uSpeed);
void Receive();
float calculatePID(float currentAngle, float dt);

void setup() {
  Serial.begin(115200);

  Serial.println("ESP32 Onewheel Initializing...");

  pinMode(VELOSTAT_PIN, INPUT_PULLUP);

  Wire.begin(MPU_SDA, MPU_SCL); 
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip - Check wiring!");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");

  // Typical settings for balancing (higher bandwidth for better reaction time)
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ); 

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
  static unsigned long iTimeSend = 0;

  // Process incoming hoverboard telemetry (run frequently)
  Receive();

  // Primary Control Loop
  if ((long)(timeNow - iTimeSend) >= 0) {
    iTimeSend = timeNow + TIME_SEND;

    // 1. Read Velostat Footpad (100ms debounce for safety)
    static bool stableFootpad = false;
    static unsigned long footpadChangeTime = 0;
    bool rawFootpad = (digitalRead(VELOSTAT_PIN) == HIGH);
    if (rawFootpad != stableFootpad) {
        if (footpadChangeTime == 0) footpadChangeTime = timeNow;
        if (timeNow - footpadChangeTime > 100) {
            stableFootpad = rawFootpad;
            footpadChangeTime = 0;
        }
    } else {
        footpadChangeTime = 0;
    }
    bool footpadPressed = stableFootpad;

    // 2. Time delta (before sensor fusion — needed by comp filter and PID)
    float dt = (timeNow - prevTime_PID) / 1000.0;
    if (dt <= 0) dt = 0.02;
    prevTime_PID = timeNow;

    // 3. Apply pending PID update from BLE (thread-safe handoff)
    if (pidUpdatePending) {
        Kp = pendingKp;
        Ki = pendingKi;
        Kd = pendingKd;
        prevError = 0;
        integral = 0;
        pidUpdatePending = false;
        Serial.printf("[PID] Applied: Kp=%.2f Ki=%.2f Kd=%.2f\n", Kp, Ki, Kd);
    }

    // 4. Read MPU6050 — complementary filter (accel + gyro fusion)
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float accelPitch = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI + 180.0;
    if (accelPitch > 180.0) accelPitch -= 360.0;

    static float pitch = 0;
    float gyroRate = g.gyro.x * 180.0 / PI;  // deg/s from gyro
    pitch = 0.98f * (pitch + gyroRate * dt) + 0.02f * accelPitch;

    int16_t uSpeed = 0;
    int16_t uSteer = 0;

    // Board safety limits (cutoff if fallen / flipped over 40 degrees)
    bool isFallen = (fabs(pitch) > 40.0);

    // 5. Compute Control Action
    if (footpadPressed && !isFallen && isArmed) {
      float pidOutput = calculatePID(pitch, dt);
      uSpeed = (int16_t)constrain(pidOutput, -1000, 1000);
    } else {
      // E-Stop / Disengage / BLE kill switch
      uSpeed = 0;
      integral = 0;
      prevError = 0;  // prevent derivative spike on re-engage
    }

    // 4. Send to Hoverboard Motherboard
    Send(uSteer, uSpeed);

    // 5. Build & send BLE telemetry packet
    {
      // Convert hoverboard raw battery voltage to actual volts
      // Hoverboard firmware reports batVoltage in 0.01V units
      float batVolts = Feedback.batVoltage / 100.0f;

      // Build status flags bitmask
      uint16_t flags = 0;
      if (footpadPressed) flags |= FLAG_FOOTPAD_LEFT | FLAG_FOOTPAD_RIGHT;
      if (isArmed)        flags |= FLAG_ARMED;
      if (Feedback.boardTemp > 600)  flags |= FLAG_OVER_TEMP;
      if (batVolts > 1.0f && batVolts < 50.0f) flags |= FLAG_LOW_BATTERY;

      TelemetryPacket pkt;
      pkt.batteryVoltage = batVolts;
      pkt.speed          = (float)abs(Feedback.speedL_meas);  // raw speed value
      pkt.pitch          = pitch;
      pkt.boardTemp      = Feedback.boardTemp;  // already in °C×10
      pkt.motorCurrentL  = Feedback.cmd1;
      pkt.motorCurrentR  = Feedback.cmd2;
      pkt.statusFlags    = flags;

      ble_sendTelemetry(pkt);
    }

    // 6. Debug output (slower rate for readability)
    static int debugCounter = 0;
    if (debugCounter++ >= 10) {
      debugCounter = 0;
      Serial.printf("Foot:%s Arm:%s Pitch:%.1f Spd:%d BatV:%d\n",
        footpadPressed ? "ON" : "OFF",
        isArmed ? "Y" : "N",
        pitch, uSpeed, Feedback.batVoltage);
    }
  }

  // Yield to FreeRTOS — prevents CPU from spinning at 100% between
  // control loop iterations, which causes significant overheating.
  delay(1);
}

float calculatePID(float currentAngle, float dt) {
  float error = targetAngle - currentAngle;
  integral += error * dt;

  // Anti-windup cap
  integral = constrain(integral, -25.0, 25.0);

  float derivative = (error - prevError) / dt;
  prevError = error;

  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
  return output;
}

void Send(int16_t uSteer, int16_t uSpeed) {
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  Serial2.write((uint8_t *) &Command, sizeof(Command)); 
}

void Receive() {
    // Drain ALL available bytes — at 115200 baud the buffer fills
    // faster than the 1ms loop can process one-at-a-time.
    while (Serial2.available()) {
        incomingByte = Serial2.read();
        bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev;

        if (bufStartFrame == START_FRAME) {
            p       = (byte *)&NewFeedback;
            *p++    = incomingBytePrev;
            *p++    = incomingByte;
            idx     = 2;
        } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {
            *p++    = incomingByte;
            idx++;
        }

        // Valid frame received?
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