// ─────────────────────────────────────────────────────────────
//  BLE GATT Server — Implementation
//  NimBLE-based BLE peripheral that streams telemetry to the
//  Onewheel PWA and accepts PID / ARM / FLASH / REBOOT commands.
// ─────────────────────────────────────────────────────────────

#include "ble_server.h"
#include <NimBLEDevice.h>
#include <Preferences.h>

// ── Internal State ──────────────────────────────────────────

static NimBLECharacteristic* pTelemetryChar  = nullptr;
static NimBLECharacteristic* pControlChar    = nullptr;
static NimBLECharacteristic* pDeviceInfoChar = nullptr;
static NimBLEServer*         pServer         = nullptr;

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
                float kp, ki, kd;
                memcpy(&kp, data + 1, 4);
                memcpy(&ki, data + 5, 4);
                memcpy(&kd, data + 9, 4);
                pendingKp = kp;
                pendingKi = ki;
                pendingKd = kd;
                pidUpdatePending = true;
                Serial.printf("[CMD] SET_PID (queued): Kp=%.2f Ki=%.2f Kd=%.2f\n", kp, ki, kd);
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
                Preferences prefs;
                prefs.begin("onewheel", false);
                prefs.putFloat("Kp", Kp);
                prefs.putFloat("Ki", Ki);
                prefs.putFloat("Kd", Kd);
                prefs.end();
                Serial.printf("  → Stored: Kp=%.2f Ki=%.2f Kd=%.2f\n", Kp, Ki, Kd);
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

// ── Public API ──────────────────────────────────────────────

void ble_init() {
    // Load saved settings from NVS (defaults are provided by variables in main.cpp)
    Preferences prefs;
    prefs.begin("onewheel", true); // true = read-only
    Kp = prefs.getFloat("Kp", Kp);
    Ki = prefs.getFloat("Ki", Ki);
    Kd = prefs.getFloat("Kd", Kd);
    Serial.printf("[NVS] Loaded PID: Kp=%.2f Ki=%.2f Kd=%.2f\n", Kp, Ki, Kd);
    prefs.end();

    // Initialize NimBLE
    NimBLEDevice::init("DIY_ONEWHEEL_ESP32");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);
    NimBLEDevice::setMTU(64);

    // Create server
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    // Create service
    NimBLEService* pService = pServer->createService(BLE_SERVICE_UUID);

    // Telemetry characteristic (Notify + Read)
    pTelemetryChar = pService->createCharacteristic(
        BLE_TELEMETRY_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );

    // Control characteristic (Write)
    pControlChar = pService->createCharacteristic(
        BLE_CONTROL_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
    );
    pControlChar->setCallbacks(new ControlCallbacks());

    // Device info characteristic (Read)
    pDeviceInfoChar = pService->createCharacteristic(
        BLE_DEVICE_INFO_CHAR_UUID,
        NIMBLE_PROPERTY::READ
    );
    const char* deviceInfo = "{\"fw\":\"1.0.0\",\"hw\":\"ESP32-DOIT-V1\",\"name\":\"DIY-OW-01\"}";
    pDeviceInfoChar->setValue((const uint8_t*)deviceInfo, strlen(deviceInfo));

    // Start service
    pService->start();

    // Start advertising
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();

    Serial.println("[BLE] Advertising started — waiting for connections...");
    Serial.printf("[BLE] Service UUID: %s\n", BLE_SERVICE_UUID);
}

void ble_sendTelemetry(const TelemetryPacket& pkt) {
    if (!pTelemetryChar) return;
    pTelemetryChar->setValue((uint8_t*)&pkt, sizeof(pkt));
    pTelemetryChar->notify();
}

bool ble_isConnected() {
    return pServer && pServer->getConnectedCount() > 0;
}
