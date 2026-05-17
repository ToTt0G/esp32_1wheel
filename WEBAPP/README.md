# DIY Onewheel — Full Project

An open-source, ESP32-powered self-balancing Onewheel with a companion Progressive Web App for real-time telemetry and wireless PID tuning over Bluetooth Low Energy.

## Architecture

```
┌──────────────────────┐        BLE (GATT)        ┌──────────────────────┐
│   ESP32 Firmware     │◄────────────────────────►│   Onewheel PWA       │
│   (PlatformIO/C++)   │  Telemetry + Commands    │   (React/TypeScript) │
└──────────┬───────────┘                          └──────────────────────┘
           │ UART (Serial2)                        Served from:
           │ 115200 baud                           1wheel.redsunsetfarm.com
┌──────────▼───────────┐
│   Hoverboard         │
│   Motor Controller   │
└──────────────────────┘
```

## Repositories

| Component | Path | Description |
|-----------|------|-------------|
| **ESP32 Firmware** | [`ESP32_OneWheel_V1/`](../../../PlatformIO/ESP32_OneWheel_V1/) | PID balancing, hoverboard UART, BLE server |
| **Companion PWA** | [`WEBAPP/Onewheel-PWA/`](WEBAPP/Onewheel-PWA/) | Dashboard, telemetry display, PID tuning UI |
| **BLE Test Firmware** | [`WEBAPP/Onewheel-PWA/esp32-reference/`](WEBAPP/Onewheel-PWA/esp32-reference/) | Fake telemetry sketch for UI development |

## How It Works

1. **MPU6050** reads pitch angle at 50 Hz
2. **PID controller** computes motor speed from angle error
3. **Hoverboard UART** sends speed commands to the motor controller
4. **Velostat footpad** gates the PID — step off = motors stop
5. **NimBLE GATT server** broadcasts telemetry to the PWA
6. **PWA** displays speed, voltage, pitch, and allows remote PID tuning + ARM/disarm

## Quick Start

### ESP32 Firmware

```bash
cd ESP32_OneWheel_V1    # or wherever PlatformIO project lives
pio run --target upload
pio device monitor -b 115200
```

See [ESP32 README](../../../PlatformIO/ESP32_OneWheel_V1/README.md) for full details.

### Companion PWA

```bash
cd WEBAPP/Onewheel-PWA
bun install
bun run dev
# Open http://localhost:5173 in Chrome
```

See [PWA README](WEBAPP/Onewheel-PWA/README.md) for full details.

## Hardware BOM

| Component | Purpose |
|-----------|---------|
| ESP32 DOIT DevKit V1 | Main controller |
| MPU6050 breakout | IMU for balance sensing |
| Velostat pressure pad | Rider detection (footpad) |
| Hoverboard mainboard | Motor driver (BLDC) |
| Hoverboard hub motor(s) | Propulsion |
| Battery (10S–15S lithium) | Power supply |

## BLE Protocol Summary

The ESP32 and PWA communicate via a custom binary GATT protocol:

- **Telemetry** (Notify, 20 bytes @ 50 Hz) — voltage, speed, pitch, temp, currents, status flags
- **Control** (Write) — SET_PID, ARM/DISARM, FLASH_CFG, REBOOT
- **Device Info** (Read) — JSON metadata string

Full protocol spec in [`WEBAPP/Onewheel-PWA/src/services/ble-protocol.ts`](WEBAPP/Onewheel-PWA/src/services/ble-protocol.ts).

## Safety

> **⚠️ This is experimental hardware.** Self-balancing vehicles are inherently dangerous. Always:
> - Test PID changes with the board elevated / wheels off the ground
> - Wear full protective gear when riding
> - Keep the ARM/DISARM toggle easily accessible on your phone
> - Never ride near traffic or at unsafe speeds

## License

This project is provided as-is for educational and personal use. Build at your own risk.
