import { AppHeader } from '../components/AppHeader';
import { BleStatusBar } from '../components/BleStatusBar';
import { BleMenuOverlay } from '../components/BleMenuOverlay';
import { PwaInstallOverlay } from '../components/PwaInstallOverlay';
import { TelemetryPanel } from '../components/TelemetryPanel';
import { Oscilloscope } from '../components/Oscilloscope';
import { FootpadToggle } from '../components/FootpadToggle';
import { PidPanel } from '../components/PidPanel';
import { useInstallPrompt } from '../hooks/useInstallPrompt';
import { useBluetooth } from '../hooks/useBluetooth';

import { useWakeLock } from '../hooks/useWakeLock';
import { useVoltageHistory } from '../hooks/useVoltageHistory';
import { StatusFlags, hasFlag } from '../services/ble-protocol';

export function Dashboard() {
    const { showInstallPrompt, install, dismiss } = useInstallPrompt();

    const {
        bleState,
        isSupported,
        telemetry,
        deviceConfig,
        terminalLog,
        isBleMenuOpen,
        openBleMenu,
        closeBleMenu,
        scan,
        disconnect,
        sendPid,
        sendArm,
    } = useBluetooth();

    const voltageHistory = useVoltageHistory(telemetry.voltage, bleState === 'connected');

    const isConnected = bleState === 'connected';
    const isArmed = hasFlag(telemetry.statusFlags, StatusFlags.ARMED);



    // Keep the screen on while connected — safety critical for riding
    useWakeLock(isConnected);

    const handleArmToggle = () => {
        sendArm(!isArmed);
    };

    const handlePidFlash = (values: { p: number; i: number; d: number, fpThr: number }) => {
        sendPid(values.p, values.i, values.d, values.fpThr);
    };

    return (
        <div className="min-h-screen bg-[#1c2317] text-[#e8f0df] font-mono p-4 pb-24 flex flex-col items-center justify-start relative [background-image:radial-gradient(#2d3826_1px,transparent_1px)] [background-size:20px_20px] touch-pan-y overflow-hidden">

            {/* Unsupported Browser Banner */}
            {!isSupported && (
                <div className="w-full max-w-sm bg-red-900/60 border border-red-700 text-red-200 text-xs p-3 mb-4 text-center uppercase tracking-widest">
                    ⚠ Web Bluetooth not supported in this browser. Use Chrome on Android/Desktop.
                </div>
            )}

            {/* PWA Install Overlay */}
            {showInstallPrompt && (
                <PwaInstallOverlay onInstall={install} onDismiss={dismiss} />
            )}

            {/* Bluetooth Menu Overlay */}
            {isBleMenuOpen && (
                <BleMenuOverlay
                    bleState={bleState}
                    terminalLog={terminalLog}
                    onClose={closeBleMenu}
                    onScan={scan}
                    onDisconnect={disconnect}
                />
            )}

            {/* Header */}
            <AppHeader title="DIY ONEWHEEL" subtitle="Command Control" />

            {/* Bluetooth Status Bar */}
            <BleStatusBar bleState={bleState} onOpenMenu={openBleMenu} />

            {/* Main Content — dims when disconnected */}
            <div className={`w-full max-w-sm flex-1 flex flex-col transition-opacity duration-1000 ${isConnected ? 'opacity-100' : 'opacity-40 pointer-events-none grayscale'}`}>

                {/* Telemetry Readouts */}
                <div className="flex flex-col mb-4">
                    <TelemetryPanel
                        speed={telemetry.speed}
                        voltage={telemetry.voltage}
                        batteryPercent={telemetry.batteryPercent}
                        pitch={telemetry.pitch}
                        isConnected={isConnected}
                    />
                </div>

                {/* Oscilloscope + Footpad Row */}
                <div className="w-full grid grid-cols-[1fr_100px] sm:grid-cols-[1fr_120px] gap-3 mb-4">
                    <Oscilloscope history={voltageHistory} />
                    <FootpadToggle isArmed={isArmed} onToggle={handleArmToggle} />
                </div>

                {/* PID Tuning */}
                <PidPanel onFlash={handlePidFlash} footpadAdc={telemetry.footpadAdc} syncedConfig={deviceConfig} />
            </div>
        </div>
    );
}
