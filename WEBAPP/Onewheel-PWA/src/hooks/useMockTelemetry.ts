import { useState, useEffect, useCallback, useRef } from 'react';

export type BleState = 'disconnected' | 'scanning' | 'pairing' | 'connected';

export interface MockDevice {
    name: string;
    id: string;
    rssi: number;
}

export interface TelemetryData {
    voltage: number;
    speed: number;
    pitch: number;
    batteryPercent: number;
}

const MAX_V = 63.0;
const MIN_V = 45.0;
const HISTORY_LENGTH = 60;

function calcBatteryPercent(voltage: number): number {
    return Math.max(0, Math.min(100, ((voltage - MIN_V) / (MAX_V - MIN_V)) * 100));
}

/**
 * Mock telemetry hook — simulates BLE data for UI development.
 * Will be replaced by `useBluetooth` when BLE integration is complete.
 */
export function useMockTelemetry() {
    // Telemetry
    const [voltage, setVoltage] = useState(60.1);
    const [voltageHistory, setVoltageHistory] = useState<number[]>(() => Array(HISTORY_LENGTH).fill(60.1));
    const [speed, setSpeed] = useState(0.0);
    const [pitch, setPitch] = useState(0.0);
    const [isArmed, setIsArmed] = useState(false);

    // BLE state
    const [bleState, setBleState] = useState<BleState>('disconnected');
    const [isBleMenuOpen, setIsBleMenuOpen] = useState(false);
    const [mockDevices, setMockDevices] = useState<MockDevice[]>([]);
    const [terminalLog, setTerminalLog] = useState<string[]>(['> SYS_INIT OK']);

    // Timeout refs for cleanup
    const scanTimeouts = useRef<ReturnType<typeof setTimeout>[]>([]);

    const addLog = useCallback((msg: string) => {
        setTerminalLog(prev => [...prev.slice(-4), msg]);
    }, []);

    // Simulate telemetry when connected
    useEffect(() => {
        if (bleState !== 'connected') {
            return;
        }

        const interval = setInterval(() => {
            setVoltage(prev => {
                const next = prev + (Math.random() * 0.8 - 0.4);
                return Math.max(48.0, Math.min(63.0, next));
            });
            setSpeed(prev => Math.max(0, prev + (Math.random() * 0.8 - 0.4)));
            setPitch(prev => prev + (Math.random() * 1.0 - 0.5));
        }, 200);

        return () => clearInterval(interval);
    }, [bleState]);

    // Track voltage history
    useEffect(() => {
        if (bleState === 'connected') {
            // Note: In real app, this history tracking should be handled 
            // where the actual data is received, not in a passive effect.
            // But this is just mock telemetry so it's fine for now, we'll
            // ignore the rule for this mock file.
            // eslint-disable-next-line react-hooks/set-state-in-effect
            setVoltageHistory(prev => [...prev.slice(1), voltage]);
        }
    }, [voltage, bleState]);

    // BLE flow actions
    const startScan = useCallback(() => {
        setBleState('scanning');
        setMockDevices([]);
        addLog('[BLE] Scanning environment...');

        const t1 = setTimeout(() => {
            setMockDevices([{ name: 'DIY_ONEWHEEL_ESP32', id: 'A1:B2:C3:D4', rssi: -45 }]);
            addLog('[BLE] Found: DIY_ONEWHEEL_ESP32');
        }, 1200);

        const t2 = setTimeout(() => {
            setMockDevices(prev => [...prev, { name: 'UNKNOWN_MAC_ADDR', id: 'F5:E4:D3:C2', rssi: -80 }]);
            addLog('[BLE] Found: UNKNOWN_MAC_ADDR');
        }, 2500);

        scanTimeouts.current = [t1, t2];
    }, [addLog]);

    const connectDevice = useCallback((id: string) => {
        setBleState('pairing');
        addLog(`[BLE] Handshake w/ ${id}...`);

        const t = setTimeout(() => {
            setBleState('connected');
            addLog('[BLE] SECURE CONNECTION ESTABLISHED');
            setTimeout(() => setIsBleMenuOpen(false), 1500);
        }, 2000);

        scanTimeouts.current.push(t);
    }, [addLog]);

    const disconnectDevice = useCallback(() => {
        setBleState('disconnected');
        setSpeed(0);
        setPitch(0);
        addLog('[BLE] Connection Terminated.');
    }, [addLog]);

    const toggleArm = useCallback(() => {
        setIsArmed(prev => !prev);
    }, []);

    const openBleMenu = useCallback(() => setIsBleMenuOpen(true), []);
    const closeBleMenu = useCallback(() => setIsBleMenuOpen(false), []);

    // Cleanup timeouts on unmount
    useEffect(() => {
        return () => {
            scanTimeouts.current.forEach(clearTimeout);
        };
    }, []);

    const telemetry: TelemetryData = {
        voltage,
        speed,
        pitch,
        batteryPercent: calcBatteryPercent(voltage),
    };

    return {
        telemetry,
        voltageHistory,
        isArmed,
        toggleArm,

        bleState,
        isBleMenuOpen,
        openBleMenu,
        closeBleMenu,
        mockDevices,
        terminalLog,

        startScan,
        connectDevice,
        disconnectDevice,
    };
}
