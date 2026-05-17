// ─────────────────────────────────────────────────────────────
//  BLE GATT Protocol Constants & Binary Helpers
//  Defines the contract between the PWA and the ESP32 firmware
// ─────────────────────────────────────────────────────────────

// ── Service & Characteristic UUIDs ──────────────────────────

/** Custom Onewheel BLE Service */
export const SERVICE_UUID = '4f4e4557-4845-454c-2d42-4c452d535643';

/** Telemetry characteristic — Notify + Read, 20 bytes at ~20 Hz */
export const TELEMETRY_CHAR_UUID = '4f4e4557-4845-454c-2d54-454c4d000000';

/** Control characteristic — Write, variable length commands */
export const CONTROL_CHAR_UUID = '4f4e4557-4845-454c-2d43-54524c000000';

/** Device info characteristic — Read, UTF-8 JSON string */
export const DEVICE_INFO_CHAR_UUID = '4f4e4557-4845-454c-2d49-4e464f000000';

/** Config characteristic — Read/Notify, 16 bytes (p, i, d, fpThr) */
export const CONFIG_CHAR_UUID = '4f4e4557-4845-454c-2d43-4f4e46000000';

// ── Telemetry Data ──────────────────────────────────────────

/**
 * Parsed telemetry packet (20 bytes, little-endian)
 * 
 * | Offset | Type     | Field              |
 * |--------|----------|--------------------|
 * | 0      | float32  | batteryVoltage (V) |
 * | 4      | float32  | speed (MPH)        |
 * | 8      | float32  | pitch (degrees)    |
 * | 12     | int16    | boardTemp (°C × 10)|
 * | 14     | int16    | motorCurrentLeft   |
 * | 16     | int16    | motorCurrentRight  |
 * | 18     | uint16   | statusFlags        |
 * | 20     | int16    | footpadAdc         |
 * | 22     | int16    | footpadThreshold   |
 */
export interface TelemetryPacket {
    batteryVoltage: number;
    speed: number;
    pitch: number;
    boardTemp: number;
    motorCurrentLeft: number;
    motorCurrentRight: number;
    statusFlags: number;
    footpadAdc: number;
    footpadThreshold: number;
}

export const TELEMETRY_PACKET_SIZE = 24;

/** Unpack a 20-byte DataView into a TelemetryPacket */
export function parseTelemetry(view: DataView): TelemetryPacket {
    const littleEndian = true;
    return {
        batteryVoltage: view.getFloat32(0, littleEndian),
        speed: view.getFloat32(4, littleEndian),
        pitch: view.getFloat32(8, littleEndian),
        boardTemp: view.getInt16(12, littleEndian) / 10,
        motorCurrentLeft: view.getInt16(14, littleEndian),
        motorCurrentRight: view.getInt16(16, littleEndian),
        statusFlags: view.getUint16(18, littleEndian),
        footpadAdc: view.getInt16(20, littleEndian),
        footpadThreshold: view.getInt16(22, littleEndian),
    };
}

// ── Status Flags ────────────────────────────────────────────

export const StatusFlags = {
    FOOTPAD_LEFT: 1 << 0,
    FOOTPAD_RIGHT: 1 << 1,
    ARMED: 1 << 2,
    OVER_TEMP: 1 << 3,
    OVER_CURRENT: 1 << 4,
    LOW_BATTERY: 1 << 5,
    CHARGING: 1 << 6,
} as const;

export function hasFlag(flags: number, flag: number): boolean {
    return (flags & flag) !== 0;
}

// ── Control Commands ────────────────────────────────────────

export const CommandId = {
    SET_PID: 0x01,
    ARM: 0x02,
    FLASH_CFG: 0x03,
    REBOOT: 0x04,
    CALIBRATE: 0x05,
} as const;

/** Build SET_PID command: [0x01][float Kp][float Ki][float Kd][float fpThr] = 17 bytes */
export function buildPidCommand(kp: number, ki: number, kd: number, footpadThreshold: number): ArrayBuffer {
    const buffer = new ArrayBuffer(17);
    const view = new DataView(buffer);
    const le = true;
    view.setUint8(0, CommandId.SET_PID);
    view.setFloat32(1, kp, le);
    view.setFloat32(5, ki, le);
    view.setFloat32(9, kd, le);
    view.setFloat32(13, footpadThreshold, le);
    return buffer;
}

/** Build ARM command: [0x02][uint8 1=arm, 0=disarm] = 2 bytes */
export function buildArmCommand(armed: boolean): ArrayBuffer {
    const buffer = new ArrayBuffer(2);
    const view = new DataView(buffer);
    view.setUint8(0, CommandId.ARM);
    view.setUint8(1, armed ? 1 : 0);
    return buffer;
}

/** Build FLASH_CFG command: [0x03] = 1 byte (save current config to NVS) */
export function buildFlashCommand(): ArrayBuffer {
    const buffer = new ArrayBuffer(1);
    new DataView(buffer).setUint8(0, CommandId.FLASH_CFG);
    return buffer;
}

/** Build REBOOT command: [0x04] = 1 byte */
export function buildRebootCommand(): ArrayBuffer {
    const buffer = new ArrayBuffer(1);
    new DataView(buffer).setUint8(0, CommandId.REBOOT);
    return buffer;
}

/** Build CALIBRATE command: [0x05] = 1 byte */
export function buildCalibrateCommand(): ArrayBuffer {
    const buffer = new ArrayBuffer(1);
    new DataView(buffer).setUint8(0, CommandId.CALIBRATE);
    return buffer;
}

// ── Device Info ─────────────────────────────────────────────

export interface DeviceInfo {
    firmware: string;
    hardware: string;
    name: string;
}

/** Parse the Device Info characteristic (UTF-8 JSON string) */
export function parseDeviceInfo(view: DataView): DeviceInfo {
    const decoder = new TextDecoder('utf-8');
    const json = decoder.decode(view.buffer);
    try {
        const data = JSON.parse(json);
        return {
            firmware: data.fw ?? 'unknown',
            hardware: data.hw ?? 'unknown',
            name: data.name ?? 'unknown',
        };
    } catch {
        return { firmware: 'unknown', hardware: 'unknown', name: 'unknown' };
    }
}

// ── Config Data ─────────────────────────────────────────────

export interface ConfigPacket {
    p: number;
    i: number;
    d: number;
    fpThr: number;
}

/** Unpack a 16-byte DataView into a ConfigPacket */
export function parseConfig(view: DataView): ConfigPacket {
    const le = true;
    return {
        p: view.getFloat32(0, le),
        i: view.getFloat32(4, le),
        d: view.getFloat32(8, le),
        fpThr: view.getFloat32(12, le),
    };
}

// ── Battery Helpers ─────────────────────────────────────────

const MAX_VOLTAGE = 63.0;
const MIN_VOLTAGE = 45.0;

export function calcBatteryPercent(voltage: number): number {
    return Math.max(0, Math.min(100, ((voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE)) * 100));
}
