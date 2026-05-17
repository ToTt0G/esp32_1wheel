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
static NimBLECharacteristic* pConfigChar     = nullptr;
static NimBLEServer*         pServer         = nullptr;

static void updateConfigChar(float p, float i, float d, float fp) {
    if (!pConfigChar) return;
    ConfigPacket pkt;
    pkt.p = p;
    pkt.i = i;
    pkt.d = d;
    pkt.fpThr = fp;
    pConfigChar->setValue((uint8_t*)&pkt, sizeof(pkt));
    pConfigChar->notify();
}

// ── Control Write Callback ──────────────────────────────────
// Fix #1/#2/#3/#5: All writes to shared variables now go through
// g_stateMutex. isArmed is buffered. Reboot uses a flag.
// FLASH_CFG takes the mutex before reading Kp/Ki/Kd/etc.

class ControlCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pChar) override {
        std::string value = pChar->getValue();
        if (value.empty()) return;

        uint8_t cmdId = value[0];
        const uint8_t* data = (const uint8_t*)value.data();
        size_t len = value.length();

        switch (cmdId) {
            case CMD_SET_PID: {
                if (len < 17) {
                    Serial.println("[CMD] SET_PID: invalid payload length (need 17 bytes)");
                    break;
                }
                float kp, ki, kd, fp;
                memcpy(&kp, data + 1,  4);
                memcpy(&ki, data + 5,  4);
                memcpy(&kd, data + 9,  4);
                memcpy(&fp, data + 13, 4);

                // Fix #1: Acquire mutex before writing the entire pending group
                // so loop() never sees a partially-updated set of gains.
                xSemaphoreTake(g_stateMutex, portMAX_DELAY);
                pendingKp                 = kp;
                pendingKi                 = ki;
                pendingKd                 = kd;
                pendingFootpadThreshold   = fp;
                pidUpdatePending          = true;
                xSemaphoreGive(g_stateMutex);

                updateConfigChar(kp, ki, kd, fp);
                Serial.printf("[CMD] SET_PID (queued): Kp=%.2f Ki=%.2f Kd=%.2f Thr=%.1f\n", kp, ki, kd, fp);
                break;
            }

            case CMD_ARM: {
                if (len < 2) break;
                bool wantArmed = (data[1] == 1);

                // Fix #2: Buffer the arm/disarm instead of writing isArmed
                // directly. loop() will apply it at a safe point.
                xSemaphoreTake(g_stateMutex, portMAX_DELAY);
                pendingIsArmed      = wantArmed;
                armedUpdatePending  = true;
                xSemaphoreGive(g_stateMutex);

                Serial.printf("[CMD] ARM request queued: %s\n", wantArmed ? "ARMED" : "DISARMED");
                break;
            }

            case CMD_FLASH_CFG: {
                Serial.println("[CMD] FLASH_CFG: Saving config to NVS...");

                // Fix #5: Take mutex before reading Kp/Ki/Kd so we can't
                // race with a concurrent SET_PID apply in loop().
                xSemaphoreTake(g_stateMutex, portMAX_DELAY);
                float snapKp   = Kp;
                float snapKi   = Ki;
                float snapKd   = Kd;
                float snapFp   = footpadThreshold;
                float snapOff  = pitchOffset;
                xSemaphoreGive(g_stateMutex);

                // NVS write is safe here because it runs on the BLE task,
                // not on the 200 Hz control loop task.
                Preferences prefs;
                prefs.begin("onewheel", false);
                prefs.putFloat("Kp",    snapKp);
                prefs.putFloat("Ki",    snapKi);
                prefs.putFloat("Kd",    snapKd);
                prefs.putFloat("fpThr", snapFp);
                prefs.putFloat("pOff",  snapOff);
                prefs.end();
                Serial.printf("  → Stored: Kp=%.2f Ki=%.2f Kd=%.2f Thr=%.1f Offset=%.1f\n",
                              snapKp, snapKi, snapKd, snapFp, snapOff);
                break;
            }

            case CMD_CALIBRATE: {
                // calibratePending is only written here and only read/cleared
                // in the 100 Hz block of loop(). The flag itself is atomic
                // on Xtensa (single byte). No mutex needed for this lone flag.
                calibratePending = true;
                Serial.println("[CMD] CALIBRATE: Zeroing pitch sensor requested");
                break;
            }

            case CMD_REBOOT: {
                // Fix #3: Never delay/restart from inside a BLE callback.
                // Set a flag; loop() will stop the motor then restart.
                rebootPending = true;
                Serial.println("[CMD] REBOOT: flagged — motor will stop, then restart");
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
    Kp               = prefs.getFloat("Kp",    Kp);
    Ki               = prefs.getFloat("Ki",    Ki);
    Kd               = prefs.getFloat("Kd",    Kd);
    footpadThreshold = prefs.getFloat("fpThr", footpadThreshold);
    pitchOffset      = prefs.getFloat("pOff",  pitchOffset);
    Serial.printf("[NVS] Loaded PID+Thr+Off: Kp=%.2f Ki=%.2f Kd=%.2f Thr=%.1f Off=%.1f\n",
                  Kp, Ki, Kd, footpadThreshold, pitchOffset);
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
    const char* deviceInfo = "{\"fw\":\"1.1.0\",\"hw\":\"ESP32-DOIT-V1\",\"name\":\"DIY-OW-01\"}";
    pDeviceInfoChar->setValue((const uint8_t*)deviceInfo, strlen(deviceInfo));

    // Config characteristic (Read + Notify)
    pConfigChar = pService->createCharacteristic(
        BLE_CONFIG_CHAR_UUID,
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
    );
    updateConfigChar(Kp, Ki, Kd, footpadThreshold);

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
    if (!pServer || pServer->getConnectedCount() == 0) return;  // skip if nobody listening
    pTelemetryChar->setValue((uint8_t*)&pkt, sizeof(pkt));
    pTelemetryChar->notify();
}

bool ble_isConnected() {
    return pServer && pServer->getConnectedCount() > 0;
}
