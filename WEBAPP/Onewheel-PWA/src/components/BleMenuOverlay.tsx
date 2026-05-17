import type { BleConnectionState } from '../services/BluetoothService';

interface BleMenuOverlayProps {
    bleState: BleConnectionState;
    terminalLog: string[];
    onClose: () => void;
    onScan: () => Promise<void>;
    onDisconnect: () => void;
}

export function BleMenuOverlay({
    bleState,
    terminalLog,
    onClose,
    onScan,
    onDisconnect,
}: BleMenuOverlayProps) {
    return (
        <div className="absolute inset-0 z-40 bg-[#11160e]/95 backdrop-blur-sm flex flex-col items-center justify-center p-4 animate-in fade-in zoom-in-95 duration-200">
            <div className="w-full max-w-sm bg-[#181d14] border border-[#ff6b00] relative shadow-[0_0_30px_rgba(255,107,0,0.1)]">
                {/* Decorative Corner Accents */}
                <CornerAccents />

                {/* Header */}
                <div className="bg-[#ff6b00] text-[#11160e] px-4 py-2 flex justify-between items-center">
                    <span className="font-bold tracking-widest uppercase text-xs">BLUETOOTH SETTINGS</span>
                    <button onClick={onClose} className="hover:text-white font-bold px-2">X</button>
                </div>

                <div className="p-4 sm:p-6 min-h-[300px] flex flex-col">
                    {/* Terminal Log */}
                    <TerminalLog entries={terminalLog} />

                    <div className="flex-1 flex flex-col">
                        {bleState === 'disconnected' && (
                            <div className="flex-1 flex flex-col">
                                <p className="text-[10px] text-[#869f6a] mb-4 leading-relaxed">
                                    Tap below to open the device picker. Select your <span className="text-[#ff6b00]">DIY_ONEWHEEL_ESP32</span> from the list.
                                </p>
                                <button
                                    onClick={onScan}
                                    className="w-full mt-auto py-3 border border-[#869f6a] text-[#869f6a] hover:bg-[#869f6a] hover:text-[#11160e] font-bold uppercase tracking-widest text-sm transition-colors"
                                >
                                    Scan for Devices
                                </button>
                            </div>
                        )}

                        {bleState === 'scanning' && (
                            <div className="flex-1 flex flex-col items-center justify-center">
                                <div className="text-sm text-[#869f6a] animate-pulse text-center">
                                    [ WAITING FOR DEVICE SELECTION... ]
                                </div>
                                <p className="text-[10px] text-[#869f6a]/70 mt-3 text-center">
                                    Select a device from the browser picker
                                </p>
                            </div>
                        )}

                        {bleState === 'connecting' && <PairingIndicator />}

                        {bleState === 'connected' && (
                            <ConnectedState onDisconnect={onDisconnect} />
                        )}
                    </div>
                </div>
            </div>
        </div>
    );
}

/* ---------- Sub-components ---------- */

function CornerAccents() {
    return (
        <>
            <div className="absolute top-0 left-0 w-4 h-4 border-t-2 border-l-2 border-[#ff6b00]" />
            <div className="absolute top-0 right-0 w-4 h-4 border-t-2 border-r-2 border-[#ff6b00]" />
            <div className="absolute bottom-0 left-0 w-4 h-4 border-b-2 border-l-2 border-[#ff6b00]" />
            <div className="absolute bottom-0 right-0 w-4 h-4 border-b-2 border-r-2 border-[#ff6b00]" />
        </>
    );
}

function TerminalLog({ entries }: { entries: string[] }) {
    return (
        <div className="bg-[#0a0d08] border border-[#2d3826] p-3 mb-6 h-24 overflow-hidden relative">
            <div className="absolute top-0 right-0 bg-[#2d3826] text-[#869f6a] text-[9px] px-1 uppercase">Syslog</div>
            <div className="flex flex-col justify-end h-full text-[10px] space-y-1">
                {entries.map((log, i) => (
                    <div key={i} className={log.includes('ESTABLISHED') ? 'text-[#ff6b00]' : 'text-[#869f6a]'}>
                        {log}
                    </div>
                ))}
            </div>
        </div>
    );
}

function PairingIndicator() {
    return (
        <div className="flex-1 flex flex-col items-center justify-center">
            <div className="w-full h-2 bg-[#11160e] border border-[#3b4731] overflow-hidden mb-4">
                <div className="h-full bg-[#ff6b00] w-1/2 animate-[pulse_0.5s_infinite]" />
            </div>
            <div className="text-sm font-bold text-[#ff6b00] animate-pulse">NEGOTIATING CONNECTION...</div>
        </div>
    );
}

function ConnectedState({ onDisconnect }: { onDisconnect: () => void }) {
    return (
        <div className="flex-1 flex flex-col">
            <div className="flex items-center justify-center py-6 border border-[#ff6b00]/30 bg-[#ff6b00]/10 mb-auto">
                <span className="text-lg font-bold text-[#ff6b00] tracking-widest">CONNECTED</span>
            </div>
            <button
                onClick={onDisconnect}
                className="w-full py-3 border border-red-900 text-red-500 hover:bg-red-900 hover:text-white font-bold uppercase tracking-widest text-sm transition-colors mt-6"
            >
                Disconnect Server
            </button>
        </div>
    );
}
