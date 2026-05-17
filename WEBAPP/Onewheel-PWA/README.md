# DIY Onewheel — Command Control PWA

A tactical HUD companion app for a custom ESP32-powered Onewheel. Connects over Bluetooth Low Energy (BLE) via the Web Bluetooth API to display real-time telemetry and tune PID settings — all from a Progressive Web App installed on your phone.

## Features

| Feature | Description |
|---------|-------------|
| **BLE Connectivity** | Connect to ESP32 via Web Bluetooth with auto-reconnect |
| **Live Telemetry** | Speed, voltage, pitch, temperature at 20 Hz |
| **Oscilloscope** | Real-time voltage waveform display |
| **PID Tuning** | Adjust Kp/Ki/Kd and flash to ESP32 NVS |
| **Arm/Disarm** | Toggle motor control remotely |
| **PWA** | Install to home screen, works offline, full-screen |
| **Wake Lock** | Screen stays on while connected (riding safety) |

## Browser Compatibility

| Platform | Browser | Supported |
|----------|---------|:---------:|
| Android | Chrome 56+ | ✅ |
| Windows/macOS | Chrome 70+ / Edge | ✅ |
| Linux | Chrome 56+ | ✅ |
| iOS | Safari | ❌ |

> **Note:** Web Bluetooth is [not supported on iOS](https://webkit.org/status/#feature-web-bluetooth). Use Chrome on Android or Desktop.

## Project Structure

```
src/
├── components/     UI components (AppHeader, TelemetryPanel, PidPanel, etc.)
├── hooks/          React hooks (useBluetooth, useWakeLock, useInstallPrompt, etc.)
├── pages/          Page compositions (Dashboard)
├── services/       BLE protocol & BluetoothService singleton
└── types/          Custom type declarations (Wake Lock API)

esp32-reference/    Reference NimBLE firmware for testing
```

## Tech Stack

- **Framework**: React 19 + TypeScript
- **Build Tool**: Vite 7
- **Styling**: Tailwind CSS v4
- **PWA**: `vite-plugin-pwa` with `injectManifest` + Workbox
- **BLE**: Web Bluetooth API + custom binary GATT protocol
- **Runtime**: Bun

---

## 🚀 Getting Started (Open Source Users)

Clone the repository and start the dev environment:

```bash
# Option A: Docker (bind mount for live sync)
docker compose up -d
# App available at http://localhost:5173

# Option B: Direct (requires Bun)
bun install
bun run dev
```

Any changes to source code will instantly reflect in the browser via Vite HMR.

### ESP32 Testing

To test BLE connectivity without real motors/sensors, flash the reference firmware:

1. Open `esp32-reference/onewheel_ble_server.ino` in Arduino IDE or PlatformIO
2. Install the **NimBLE-Arduino** library
3. Flash to any ESP32 board
4. Open the PWA in Chrome → Bluetooth → Scan for Devices
5. Select `DIY_ONEWHEEL_ESP32` from the browser picker

The reference firmware sends fake telemetry at 20 Hz and prints received commands to Serial Monitor.

---

## 🏗 Developer Deployment (Server)

For deploying to the production server at `1wheel.redsunsetfarm.com`.

| Question | Development | Production |
|----------|-------------|------------|
| Where does **Code** live? | Bind Mount (`./:/app`) | Inside the Image (`COPY . .`) |
| Where does **Data** live? | Docker Volume (ephemeral) | N/A (static site, no DB) |
| Where do **Secrets** live? | `.env` (local) | `/mnt/code/project/.env` (manual) |

### Deployment Steps

1. Code on your laptop — changes sync to NAS via Synology Drive.
2. SSH into the server: `ssh ryder@192.168.1.XX`
3. Build and deploy:
   ```bash
   cd /mnt/code/ESP32_Onewheel/WEBAPP/Onewheel-PWA
   docker compose -f docker-compose.prod.yml up -d --build
   ```

> This is a static PWA — no database or `/mnt/data/` mount is required.

---

## BLE Protocol

The app communicates with the ESP32 via a custom binary GATT protocol:

| Characteristic | UUID | Direction | Description |
|---------------|------|-----------|-------------|
| Telemetry | `4f4e4557-...-454c4d000000` | ESP32 → PWA (Notify) | 20-byte packet at 20 Hz |
| Control | `4f4e4557-...-54524c000000` | PWA → ESP32 (Write) | PID, ARM, FLASH commands |
| Device Info | `4f4e4557-...-4e464f000000` | ESP32 → PWA (Read) | JSON device metadata |

See [`src/services/ble-protocol.ts`](src/services/ble-protocol.ts) for the full spec.
