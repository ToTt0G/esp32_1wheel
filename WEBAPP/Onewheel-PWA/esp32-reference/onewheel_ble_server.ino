// ─────────────────────────────────────────────────────────────
//  ESP32 Onewheel — BLE GATT Server Reference Sketch
//  
//  NimBLE-Arduino | Sends fake telemetry at 20 Hz
//  Accepts PID / ARM / FLASH / REBOOT commands via write callback
//  Prints received commands to Serial Monitor for debugging
//
//  Dependencies:
//    - NimBLE-Arduino (install via PlatformIO or Arduino Library Manager)
//
//  This is a TESTING REFERENCE — flash it to any ESP32 to immediately
//  test the PWA ↔ ESP32 connection without motor/sensor firmware.
// ─────────────────────────────────────────────────────────────

#include <NimBLEDevice.h>

// ── Service & Characteristic UUIDs (must match ble-protocol.ts) ──

#define SERVICE_UUID          "4f4e4557-4845-454c-2d42-4c452d535643"
#define TELEMETRY_CHAR_UUID   "4f4e4557-4845-454c-2d54-454c4d000000"
#define CONTROL_CHAR_UUID     "4f4e4557-4845-454c-2d43-54524c000000"
#define DEVICE_INFO_CHAR_UUID "4f4e4557-4845-454c-2d49-4e464f000000"

// ── Command IDs ─────────────────────────────────────────────

#define CMD_SET_PID   0x01
#define CMD_ARM       0x02
#define CMD_FLASH_CFG 0x03
#define CMD_REBOOT    0x04

// ── Status Flags ────────────────────────────────────────────

#define FLAG_FOOTPAD_LEFT   (1 << 0)
#define FLAG_FOOTPAD_RIGHT  (1 << 1)
#define FLAG_ARMED          (1 << 2)
#define FLAG_OVER_TEMP      (1 << 3)
#define FLAG_OVER_CURRENT   (1 << 4)
#define FLAG_LOW_BATTERY    (1 << 5)
#define FLAG_CHARGING       (1 << 6)

// ── Globals ─────────────────────────────────────────────────

NimBLECharacteristic* pTelemetryChar = nullptr;
NimBLECharacteristic* pControlChar   = nullptr;
NimBLECharacteristic* pDeviceInfoChar = nullptr;

bool isArmed = false;
float pidKp = 2.4f, pidKi = 2.9f, pidKd = 3.4f;

// Fake telemetry values
float fakeVoltage = 58.0f;
float fakeSpeed   = 0.0f;
float fakePitch   = 0.0f;
int16_t fakeTemp  = 350; // 35.0°C × 10

// ── Telemetry Packet Structure (20 bytes, little-endian) ────

#pragma pack(push, 1)
struct TelemetryPacket {
    float batteryVoltage;    // offset 0
    float speed;             // offset 4
    float pitch;             // offset 8
    int16_t boardTemp;       // offset 12 (°C × 10)
    int16_t motorCurrentL;   // offset 14
    int16_t motorCurrentR;   // offset 16
    uint16_t statusFlags;    // offset 18
};
#pragma pack(pop)

static_assert(sizeof(TelemetryPacket) == 20, "Telemetry packet must be exactly 20 bytes");

// ── Control Write Callback ──────────────────────────────────

class ControlCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar) override {
        std::string value = pChar->getValue();
        if (value.empty()) return;

        uint8_t cmdId = value[0];
        const uint8_t* data = (const uint8_t*)value.data();
        size_t len = value.length();

        switch (cmdId) {
            case CMD_SET_PID: {
                if (len < 13) {
                    Serial.println("[CMD] SET_PID: invalid payload length");
                    break;
                }
                memcpy(&pidKp, data + 1, 4);
                memcpy(&pidKi, data + 5, 4);
                memcpy(&pidKd, data + 9, 4);
                Serial.printf("[CMD] SET_PID: Kp=%.2f Ki=%.2f Kd=%.2f\n", pidKp, pidKi, pidKd);
                break;
            }
            case CMD_ARM: {
                if (len < 2) break;
                isArmed = data[1] == 1;
                Serial.printf("[CMD] ARM: %s\n", isArmed ? "ARMED" : "DISARMED");
                break;
            }
            case CMD_FLASH_CFG: {
                Serial.println("[CMD] FLASH_CFG: Saving config to NVS...");
                // In real firmware: save PID values to NVS/EEPROM
                Serial.printf("  → Stored: Kp=%.2f Ki=%.2f Kd=%.2f\n", pidKp, pidKi, pidKd);
                break;
            }
            case CMD_REBOOT: {
                Serial.println("[CMD] REBOOT: Restarting in 1 second...");
                delay(1000);
                ESP.restart();
                break;
            }
            default:
                Serial.printf("[CMD] Unknown command: 0x%02X\n", cmdId);
                break;
        }
    }
};

// ── Connection Callbacks ────────────────────────────────────

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) override {
        Serial.println("[BLE] Client connected");
    }

    void onDisconnect(NimBLEServer* pServer) override {
        Serial.println("[BLE] Client disconnected — restarting advertising");
        NimBLEDevice::startAdvertising();
    }
};

// ── Setup ───────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== DIY ONEWHEEL BLE Server ===");

    // Initialize NimBLE
    NimBLEDevice::init("DIY_ONEWHEEL_ESP32");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    NimBLEDevice::setMTU(64);

    // Create server
    NimBLEServer* pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    // Create service
    NimBLEService* pService = pServer->createService(SERVICE_UUID);

    // ── Telemetry characteristic (Notify + Read) ──
    pTelemetryChar = pService->createCharacteristic(
        TELEMETRY_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );

    // ── Control characteristic (Write) ──
    pControlChar = pService->createCharacteristic(
        CONTROL_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pControlChar->setCallbacks(new ControlCallbacks());

    // ── Device info characteristic (Read) ──
    pDeviceInfoChar = pService->createCharacteristic(
        DEVICE_INFO_CHAR_UUID,
        NIMBLE_PROPERTY::READ
    );
    // Set device info JSON
    const char* deviceInfo = "{\"fw\":\"0.1.0\",\"hw\":\"ESP32-S3\",\"name\":\"OW-TEST-01\"}";
    pDeviceInfoChar->setValue((const uint8_t*)deviceInfo, strlen(deviceInfo));

    // Start service
    pService->start();

    // Start advertising
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();

    Serial.println("[BLE] Advertising started — waiting for connections...");
    Serial.printf("[BLE] Service UUID: %s\n", SERVICE_UUID);
}

// ── Loop: Send fake telemetry at ~20 Hz ─────────────────────

unsigned long lastTelemetryMs = 0;
const unsigned long TELEMETRY_INTERVAL_MS = 50; // 20 Hz

void loop() {
    unsigned long now = millis();

    if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
        lastTelemetryMs = now;

        // Simulate telemetry drift
        fakeVoltage += (float)(random(-20, 20)) / 100.0f;
        fakeVoltage = constrain(fakeVoltage, 48.0f, 63.0f);

        fakeSpeed += (float)(random(-20, 20)) / 100.0f;
        fakeSpeed = constrain(fakeSpeed, 0.0f, 25.0f);

        fakePitch += (float)(random(-25, 25)) / 100.0f;
        fakePitch = constrain(fakePitch, -15.0f, 15.0f);

        // Build status flags
        uint16_t flags = 0;
        flags |= FLAG_FOOTPAD_LEFT;   // Always active in test mode
        flags |= FLAG_FOOTPAD_RIGHT;
        if (isArmed) flags |= FLAG_ARMED;
        if (fakeTemp > 600) flags |= FLAG_OVER_TEMP;
        if (fakeVoltage < 50.0f) flags |= FLAG_LOW_BATTERY;

        // Build telemetry packet
        TelemetryPacket pkt;
        pkt.batteryVoltage = fakeVoltage;
        pkt.speed          = fakeSpeed;
        pkt.pitch          = fakePitch;
        pkt.boardTemp      = fakeTemp;
        pkt.motorCurrentL  = (int16_t)(random(100, 500));
        pkt.motorCurrentR  = (int16_t)(random(100, 500));
        pkt.statusFlags    = flags;

        // Send via notify
        pTelemetryChar->setValue((uint8_t*)&pkt, sizeof(pkt));
        pTelemetryChar->notify();
    }
}
