import { useState, useEffect, useCallback, useRef } from 'react';
import { bluetoothService, type BleConnectionState } from '../services/BluetoothService';
import {
    buildPidCommand,
    buildArmCommand,
    buildFlashCommand,
    calcBatteryPercent,
    type TelemetryPacket,
    type DeviceInfo,
    type ConfigPacket,
} from '../services/ble-protocol';

// ── Public Interface ────────────────────────────────────────

export interface BluetoothTelemetry {
    voltage: number;
    speed: number;
    pitch: number;
    batteryPercent: number;
    boardTemp: number;
    motorCurrentLeft: number;
    motorCurrentRight: number;
    statusFlags: number;
    footpadAdc: number;
    footpadThreshold: number;
}

export interface UseBluetooth {
    // State
    bleState: BleConnectionState;
    isSupported: boolean;
    telemetry: BluetoothTelemetry;
    deviceInfo: DeviceInfo | null;
    deviceConfig: ConfigPacket | null;
    terminalLog: string[];

    // BLE menu
    isBleMenuOpen: boolean;
    openBleMenu: () => void;
    closeBleMenu: () => void;

    // Actions
    scan: () => Promise<void>;
    disconnect: () => void;
    sendPid: (kp: number, ki: number, kd: number, footpadThreshold: number) => Promise<void>;
    sendArm: (armed: boolean) => Promise<void>;
    flashSettings: () => Promise<void>;
}

// ── Default Values ──────────────────────────────────────────

const DEFAULT_TELEMETRY: BluetoothTelemetry = {
    voltage: 0,
    speed: 0,
    pitch: 0,
    batteryPercent: 0,
    boardTemp: 0,
    motorCurrentLeft: 0,
    motorCurrentRight: 0,
    statusFlags: 0,
    footpadAdc: 4095,
    footpadThreshold: 10,
};

const MAX_LOG_ENTRIES = 5;

// ── Hook ────────────────────────────────────────────────────

export function useBluetooth(): UseBluetooth {
    const [bleState, setBleState] = useState<BleConnectionState>(bluetoothService.state);
    const [telemetry, setTelemetry] = useState<BluetoothTelemetry>(DEFAULT_TELEMETRY);
    const [deviceInfo, setDeviceInfo] = useState<DeviceInfo | null>(bluetoothService.deviceInfo);
    const [deviceConfig, setDeviceConfig] = useState<ConfigPacket | null>(bluetoothService.deviceConfig);
    const [terminalLog, setTerminalLog] = useState<string[]>(['> SYS_INIT OK']);
    const [isBleMenuOpen, setIsBleMenuOpen] = useState(false);

    // Use ref to batch high-frequency telemetry updates via rAF
    const latestPacket = useRef<TelemetryPacket | null>(null);
    const rafId = useRef<number>(0);

    // Subscribe to service events
    useEffect(() => {
        const unsubState = bluetoothService.onStateChange((state) => {
            setBleState(state);
            if (state === 'connected') {
                setDeviceInfo(bluetoothService.deviceInfo);
                setDeviceConfig(bluetoothService.deviceConfig);
            }
            if (state === 'disconnected') {
                setTelemetry(DEFAULT_TELEMETRY);
                setDeviceConfig(null);
            }
        });

        const unsubConfig = bluetoothService.onConfigChange((config) => {
            setDeviceConfig(config);
        });

        const unsubTelemetry = bluetoothService.onTelemetry((packet) => {
            // Buffer the packet — we'll apply it in the next rAF
            latestPacket.current = packet;

            if (!rafId.current) {
                rafId.current = requestAnimationFrame(() => {
                    const p = latestPacket.current;
                    if (p) {
                        setTelemetry({
                            voltage: p.batteryVoltage,
                            speed: p.speed,
                            pitch: p.pitch,
                            batteryPercent: calcBatteryPercent(p.batteryVoltage),
                            boardTemp: p.boardTemp,
                            motorCurrentLeft: p.motorCurrentLeft,
                            motorCurrentRight: p.motorCurrentRight,
                            statusFlags: p.statusFlags,
                            footpadAdc: p.footpadAdc,
                            footpadThreshold: p.footpadThreshold,
                        });
                    }
                    rafId.current = 0;
                });
            }
        });

        const unsubLog = bluetoothService.onLog((msg) => {
            setTerminalLog(prev => [...prev.slice(-(MAX_LOG_ENTRIES - 1)), msg]);
        });

        const unsubError = bluetoothService.onError((msg) => {
            setTerminalLog(prev => [...prev.slice(-(MAX_LOG_ENTRIES - 1)), msg]);
        });

        return () => {
            unsubState();
            unsubConfig();
            unsubTelemetry();
            unsubLog();
            unsubError();
            if (rafId.current) cancelAnimationFrame(rafId.current);
        };
    }, []);

    // Actions
    const scan = useCallback(async () => {
        await bluetoothService.scan();
    }, []);

    const disconnect = useCallback(() => {
        bluetoothService.disconnect();
    }, []);

    const sendPid = useCallback(async (kp: number, ki: number, kd: number, footpadThreshold: number) => {
        await bluetoothService.sendCommand(buildPidCommand(kp, ki, kd, footpadThreshold));
    }, []);

    const sendArm = useCallback(async (armed: boolean) => {
        await bluetoothService.sendCommand(buildArmCommand(armed));
    }, []);

    const flashSettings = useCallback(async () => {
        await bluetoothService.sendCommand(buildFlashCommand());
    }, []);

    const openBleMenu = useCallback(() => setIsBleMenuOpen(true), []);
    const closeBleMenu = useCallback(() => setIsBleMenuOpen(false), []);

    return {
        bleState,
        isSupported: bluetoothService.isSupported,
        telemetry,
        deviceInfo,
        deviceConfig,
        terminalLog,

        isBleMenuOpen,
        openBleMenu,
        closeBleMenu,

        scan,
        disconnect,
        sendPid,
        sendArm,
        flashSettings,
    };
}
