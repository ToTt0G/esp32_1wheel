interface TelemetryPanelProps {
    speed: number;
    voltage: number;
    batteryPercent: number;
    pitch: number;
    isConnected: boolean;
}

export function TelemetryPanel({ speed, voltage, batteryPercent, pitch, isConnected }: TelemetryPanelProps) {
    return (
        <div className="w-full bg-[#181d14] border-2 border-[#3b4731] relative mb-4">
            {/* Corner Accents */}
            <div className="absolute top-0 left-0 w-3 h-3 border-t-2 border-l-2 border-[#869f6a]" />
            <div className="absolute top-0 right-0 w-3 h-3 border-t-2 border-r-2 border-[#869f6a]" />
            <div className="absolute bottom-0 left-0 w-3 h-3 border-b-2 border-l-2 border-[#869f6a]" />
            <div className="absolute bottom-0 right-0 w-3 h-3 border-b-2 border-r-2 border-[#869f6a]" />

            {/* Speed Readout */}
            <div className="p-4 sm:p-6 text-center border-b border-[#2d3826]">
                <div className="flex justify-between items-center mb-2 px-2">
                    <span className="text-[10px] text-[#869f6a] tracking-[0.2em] uppercase">Ground Velocity</span>
                    <span className={`text-[9px] bg-[#2d3826] px-1.5 py-0.5 rounded-sm ${isConnected ? 'text-[#e8f0df]' : 'text-[#869f6a]/30'}`}>
                        {isConnected ? 'LOCKED' : 'WAIT'}
                    </span>
                </div>
                <div className="flex justify-center items-baseline gap-2">
                    <span className="text-5xl sm:text-6xl font-bold tracking-tight text-[#f4ffea] tabular-nums">
                        {speed.toFixed(1)}
                    </span>
                    <span className="text-sm sm:text-lg text-[#869f6a]">KM/H</span>
                </div>
            </div>

            {/* Secondary Readouts */}
            <div className="grid grid-cols-2 divide-x divide-[#2d3826]">
                <BatteryReadout voltage={voltage} batteryPercent={batteryPercent} />
                <PitchReadout pitch={pitch} />
            </div>
        </div>
    );
}

/* ---------- Sub-components ---------- */

function BatteryReadout({ voltage, batteryPercent }: { voltage: number; batteryPercent: number }) {
    const filledBars = Math.round(batteryPercent / 10);

    return (
        <div className="p-3 sm:p-4 flex flex-col items-center">
            <div className="text-[10px] text-[#869f6a] uppercase tracking-widest mb-1 w-full text-center flex justify-between">
                <span>Power Rsv</span>
                <span className="text-[#ff6b00]">{voltage.toFixed(1)}V</span>
            </div>
            <div className="text-2xl font-bold tabular-nums mt-1">{Math.round(batteryPercent)}%</div>
            {/* Mini power bar */}
            <div className="w-full h-1 mt-2 bg-[#11160e] flex gap-px">
                {Array.from({ length: 10 }, (_, i) => (
                    <div key={i} className={`flex-1 ${i < filledBars ? 'bg-[#ff6b00]' : 'bg-transparent'}`} />
                ))}
            </div>
        </div>
    );
}

function PitchReadout({ pitch }: { pitch: number }) {
    return (
        <div className="p-3 sm:p-4 flex flex-col items-center justify-between">
            <div className="text-[10px] text-[#869f6a] uppercase tracking-widest w-full text-center mb-2">Pitch Att</div>
            <div className="flex items-center justify-center gap-3 w-full mt-auto">
                <div className="text-2xl font-bold text-[#ff6b00] tabular-nums w-16 text-right">{pitch.toFixed(1)}&deg;</div>
                <div
                    className="w-10 h-10 rounded-full border border-dashed border-[#546a40] relative flex items-center justify-center transition-transform duration-300 ease-out shrink-0"
                    style={{ transform: `rotate(${pitch}deg)` }}
                >
                    <div className="w-8 h-0.5 bg-[#ff6b00] shadow-[0_0_5px_rgba(255,107,0,0.5)]" />
                    <div className="absolute w-2 h-2 rounded-none bg-[#181d14] border border-[#ff6b00]" />
                </div>
            </div>
        </div>
    );
}
