# DIY Onewheel — ESP32 Firmware

PlatformIO-based firmware for a custom ESP32-powered Onewheel. Uses an MPU6050 IMU for PID-based self-balancing, communicates with a hoverboard motor controller over UART, and exposes real-time telemetry via BLE to the [companion PWA](../WEBAPP/Onewheel-PWA/).

## Features

| Feature | Description |
|---------|-------------|
| **PID Balancing** | Real-time self-balancing using MPU6050 pitch angle |
| **Hoverboard UART** | Sends speed commands and reads feedback via Serial2 |
| **BLE Telemetry** | Streams voltage, speed, pitch, temperature at ~50 Hz |
| **Remote PID Tuning** | Receive Kp/Ki/Kd values over BLE from the PWA |
| **Remote ARM/Disarm** | Software kill switch toggled from the PWA |
| **Safety Cutoffs** | Auto-disengage on >40° tilt or footpad release |

## Hardware

| Component | Pin(s) | Notes |
|-----------|--------|-------|
| MPU6050 IMU | SDA=23, SCL=22 | I²C — pitch angle for PID |
| Velostat Footpad | GPIO 21 | Pulled up — HIGH when pressed |
| Hoverboard Controller | TX=17, RX=16 | UART @ 115200 baud (Serial2) |
| Onboard LED | LED_BUILTIN | Status indicator |

## Project Structure

```
├── include/
│   └── ble_server.h        BLE types, UUIDs, packet struct, API
├── src/
│   ├── main.cpp            Control loop, PID, hoverboard comms
│   └── ble_server.cpp      NimBLE GATT server implementation
└── platformio.ini          Build config & dependencies
```

## Dependencies

| Library | Version | Purpose |
|---------|---------|---------|
| [Adafruit MPU6050](https://github.com/adafruit/Adafruit_MPU6050) | ^2.2.6 | IMU driver |
| [Adafruit Unified Sensor](https://github.com/adafruit/Adafruit_Sensor) | ^1.1.14 | Sensor abstraction |
| [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino) | ^1.4.3 | Lightweight BLE stack |

## Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/) (VS Code extension or CLI)
- ESP32 DOIT DevKit V1 (or compatible)
- USB cable for flashing

### Build & Flash

```bash
# Build only
pio run -e esp32doit-devkit-v1

# Build and upload
pio run --target upload -e esp32doit-devkit-v1

# Serial monitor (115200 baud)
pio device monitor -b 115200
```

### Expected Serial Output

```
ESP32 Onewheel Initializing...
MPU6050 Found!
[BLE] Advertising started — waiting for connections...
[BLE] Service UUID: 4f4e4557-4845-454c-2d42-4c452d535643
Foot:OFF Arm:Y Pitch:0.3 Spd:0 BatV:5800
```

## BLE Protocol

The firmware implements a custom GATT server that the companion PWA connects to:

| Characteristic | UUID | Direction | Description |
|---------------|------|-----------|-------------|
| Telemetry | `...454c4d000000` | ESP32 → PWA (Notify) | 20-byte binary packet |
| Control | `...54524c000000` | PWA → ESP32 (Write) | PID, ARM, FLASH, REBOOT |
| Device Info | `...4e464f000000` | ESP32 → PWA (Read) | JSON device metadata |

### Telemetry Packet (20 bytes, little-endian)

| Offset | Type | Field |
|--------|------|-------|
| 0 | float32 | Battery voltage (V) |
| 4 | float32 | Speed (raw) |
| 8 | float32 | Pitch (degrees) |
| 12 | int16 | Board temp (°C × 10) |
| 14 | int16 | Motor current L |
| 16 | int16 | Motor current R |
| 18 | uint16 | Status flags (bitmask) |

### Control Commands

| ID | Name | Payload |
|----|------|---------|
| `0x01` | SET_PID | `[Kp: f32][Ki: f32][Kd: f32]` (13 bytes) |
| `0x02` | ARM | `[0\|1: u8]` (2 bytes) |
| `0x03` | FLASH_CFG | None (1 byte) |
| `0x04` | REBOOT | None (1 byte) |

## PID Tuning

Default values: `Kp=60.0, Ki=0.5, Kd=2.0`. These can be overridden at runtime via BLE from the PWA. The `FLASH_CFG` command is stubbed — implement NVS/Preferences writes to persist across reboots.

> **⚠️ Warning:** PID tuning on a balancing vehicle is safety-critical. Always test with the board secured/elevated before riding.
