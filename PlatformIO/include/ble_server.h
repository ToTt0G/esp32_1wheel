// ─────────────────────────────────────────────────────────────
//  BLE GATT Server — Header
//  Defines the protocol contract matching the Onewheel PWA
//  (ble-protocol.ts / BluetoothService.ts)
// ─────────────────────────────────────────────────────────────

#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ── Service & Characteristic UUIDs (must match ble-protocol.ts) ──

#define BLE_SERVICE_UUID          "4f4e4557-4845-454c-2d42-4c452d535643"
#define BLE_TELEMETRY_CHAR_UUID   "4f4e4557-4845-454c-2d54-454c4d000000"
#define BLE_CONTROL_CHAR_UUID     "4f4e4557-4845-454c-2d43-54524c000000"
#define BLE_DEVICE_INFO_CHAR_UUID "4f4e4557-4845-454c-2d49-4e464f000000"
#define BLE_CONFIG_CHAR_UUID      "4f4e4557-4845-454c-2d43-4f4e46000000"

// ── Command IDs ─────────────────────────────────────────────

#define CMD_SET_PID   0x01
#define CMD_ARM       0x02
#define CMD_FLASH_CFG 0x03
#define CMD_REBOOT    0x04
#define CMD_CALIBRATE 0x05

// ── Status Flags (bitmask) ──────────────────────────────────

#define FLAG_FOOTPAD_LEFT   (1 << 0)
#define FLAG_FOOTPAD_RIGHT  (1 << 1)
#define FLAG_ARMED          (1 << 2)
#define FLAG_OVER_TEMP      (1 << 3)
#define FLAG_OVER_CURRENT   (1 << 4)
#define FLAG_LOW_BATTERY    (1 << 5)
#define FLAG_CHARGING       (1 << 6)

// ── Telemetry Packet (24 bytes, little-endian, packed) ──────

#pragma pack(push, 1)
struct TelemetryPacket {
    float    batteryVoltage;   // offset 0  — Volts
    float    speed;            // offset 4  — km/h
    float    pitch;            // offset 8  — degrees
    int16_t  boardTemp;        // offset 12 — °C × 10
    int16_t  motorCurrentL;    // offset 14
    int16_t  motorCurrentR;    // offset 16
    uint16_t statusFlags;      // offset 18
    int16_t  footpadAdc;       // offset 20
    int16_t  footpadThreshold; // offset 22
};
#pragma pack(pop)

static_assert(sizeof(TelemetryPacket) == 24, "Telemetry packet must be exactly 24 bytes");

// ── Config Packet (16 bytes, little-endian, packed) ─────────

#pragma pack(push, 1)
struct ConfigPacket {
    float p;      // PID Proportional
    float i;      // PID Integral
    float d;      // PID Derivative
    float fpThr;  // Footpad Threshold
};
#pragma pack(pop)

static_assert(sizeof(ConfigPacket) == 16, "Config packet must be exactly 16 bytes");

// ── Mutex — protects all cross-task shared state ────────────
// Fix #1/#2/#5: All BLE callbacks acquire this before touching
// any variable also used by loop(). loop() acquires before reading
// the pending buffers.

extern SemaphoreHandle_t g_stateMutex;

// ── Shared State (owned by main.cpp, written by BLE callbacks) ──

extern float Kp, Ki, Kd, footpadThreshold, pitchOffset;
extern bool  isArmed;

// ── Thread-safe pending buffers (BLE task → loop task) ──────
// Fix #1: Full group is written atomically under g_stateMutex,
//         then consumed by loop() under the same mutex.
// Fix #2: isArmed is now buffered instead of written directly.
// Fix #3: rebootPending is consumed in loop() so motor can be
//         stopped before the restart.

extern volatile bool  pidUpdatePending;
extern volatile float pendingKp, pendingKi, pendingKd, pendingFootpadThreshold;

extern volatile bool  armedUpdatePending;
extern volatile bool  pendingIsArmed;

extern volatile bool  rebootPending;
extern volatile bool  calibratePending;

// ── NVS dirty flags (deferred writes — Fix #4) ──────────────
// Set by loop() when a value changes; a low-priority task
// (or idle check) performs the actual flash write.

extern volatile bool nvsPidDirty;

// ── Public API ──────────────────────────────────────────────

/** Initialize NimBLE, create GATT service, start advertising */
void ble_init();

/** Send a telemetry packet via BLE notify (call from control loop) */
void ble_sendTelemetry(const TelemetryPacket& pkt);

/** Returns true if at least one BLE client is connected */
bool ble_isConnected();
