import type { BleConnectionState } from '../services/BluetoothService';

interface BleStatusBarProps {
    bleState: BleConnectionState;
    onOpenMenu: () => void;
}

export function BleStatusBar({ bleState, onOpenMenu }: BleStatusBarProps) {
    const isConnected = bleState === 'connected';

    return (
        <button
            onClick={onOpenMenu}
            className={`w-full max-w-sm flex justify-between items-center p-2 border mb-4 rounded-sm shadow-md transition-all outline-none focus-visible:ring-1 focus-visible:ring-[#ff6b00] ${isConnected
                    ? 'bg-[#11160e] border-[#3b4731] hover:bg-[#181d14]'
                    : 'bg-[#2d0000]/60 border-red-900 hover:bg-[#3d0000]'
                }`}
        >
            <div className="flex flex-col items-start">
                <span className="text-[10px] text-[#869f6a] tracking-widest uppercase mb-0.5">Bluetooth</span>
                <span className={`text-xs sm:text-sm font-bold tracking-wider ${isConnected ? 'text-[#e8f0df]' : 'text-red-500'}`}>
                    {isConnected ? 'DIY ONEWHEEL [ESP32]' : 'DISCONNECTED'}
                </span>
            </div>
            <div className="flex flex-col items-end">
                <div className="flex gap-1 mb-1">
                    {isConnected ? (
                        <>
                            <div className="w-1.5 h-3 bg-[#ff6b00] animate-pulse" />
                            <div className="w-1.5 h-3 bg-[#ff6b00] animate-pulse" style={{ animationDelay: '100ms' }} />
                            <div className="w-1.5 h-3 bg-[#ff6b00] animate-pulse" style={{ animationDelay: '200ms' }} />
                            <div className="w-1.5 h-3 bg-[#3b4731]" />
                        </>
                    ) : (
                        <>
                            <div className="w-1.5 h-3 bg-red-900/50" />
                            <div className="w-1.5 h-3 bg-red-900/50" />
                            <div className="w-1.5 h-3 bg-red-900/50" />
                            <div className="w-1.5 h-3 bg-red-900/50" />
                        </>
                    )}
                </div>
                <span className={`text-[9px] uppercase tracking-widest ${isConnected ? 'text-[#ff6b00]' : 'text-red-600'}`}>
                    {isConnected ? 'Signal STR' : 'OFFLINE'}
                </span>
            </div>
        </button>
    );
}
