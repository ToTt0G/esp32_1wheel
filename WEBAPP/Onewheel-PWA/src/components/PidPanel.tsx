import { useState, useCallback, useEffect } from 'react';
import { bluetoothService } from '../services/BluetoothService';
import { type ConfigPacket } from '../services/ble-protocol';

interface PidValues {
    p: number;
    i: number;
    d: number;
    fpThr: number;
}

interface PidPanelProps {
    initialValues?: PidValues;
    onFlash?: (values: PidValues) => void;
    footpadAdc?: number;
    syncedConfig?: ConfigPacket | null;
}

const DEFAULT_PID: PidValues = { p: 60.0, i: 0.5, d: 2.0, fpThr: 10.0 };

export function PidPanel({ initialValues = DEFAULT_PID, onFlash, footpadAdc, syncedConfig }: PidPanelProps) {
    const [pid, setPid] = useState<PidValues>(initialValues);

    // Auto-sync when device provides its current config
    useEffect(() => {
        if (syncedConfig) {
            setPid({
                p: syncedConfig.p,
                i: syncedConfig.i,
                d: syncedConfig.d,
                fpThr: syncedConfig.fpThr
            });
        }
    }, [syncedConfig]);

    const updateValue = useCallback((key: keyof PidValues, value: number) => {
        setPid(prev => ({ ...prev, [key]: value }));
    }, []);

    const handleFlash = useCallback(() => {
        onFlash?.(pid);
    }, [pid, onFlash]);

    return (
        <div className="w-full flex-1 bg-[#181d14] border border-[#3b4731] p-3 sm:p-4 flex flex-col">
            <div className="flex justify-between items-center mb-4 border-b border-[#2d3826] pb-2">
                <span className="text-xs font-bold uppercase tracking-widest text-[#e8f0df]">Tuning Profile [A]</span>
                {footpadAdc !== undefined && (
                    <span className="text-[10px] text-[#869f6a] font-mono tracking-wider transition-colors duration-75">
                        ADC: <span className="text-[#ff6b00]">{footpadAdc}</span>
                    </span>
                )}
                <div className="bg-[#ff6b00] w-2 h-2 rounded-none ml-2" />
            </div>

            <div className="space-y-5 flex-1 p-1 -ml-1">
                <PidSlider label="Kp (Prop)" value={pid.p} onChange={v => updateValue('p', v)} min={0} max={200} step={1} />
                <PidSlider label="Ki (Intg)" value={pid.i} onChange={v => updateValue('i', v)} min={0} max={10} step={0.1} />
                <PidSlider label="Kd (Derv)" value={pid.d} onChange={v => updateValue('d', v)} min={0} max={20} step={0.1} />
                <PidSlider label="Footpad Thr" value={pid.fpThr} onChange={v => updateValue('fpThr', v)} min={1} max={400} step={1} />
            </div>

            <button
                onClick={handleFlash}
                className="w-full mt-4 bg-[#ff6b00] hover:bg-[#ff8c3a] focus-visible:bg-[#ff8c3a] text-[#11160e] font-bold py-3 text-sm uppercase tracking-[0.2em] transition-colors outline-none focus-visible:ring-2 focus-visible:ring-white"
            >
                Flash Settings
            </button>

            <button
                onClick={() => bluetoothService.calibrate()}
                className="w-full mt-2 bg-transparent border border-[#869f6a] hover:bg-[#869f6a]/10 text-[#869f6a] hover:text-[#e8f0df] font-bold py-2 text-[10px] uppercase tracking-[0.2em] transition-all outline-none focus-visible:ring-1 focus-visible:ring-[#869f6a]"
            >
                Calibrate Pitch (Zero)
            </button>
        </div>
    );
}

/* ---------- Sub-component ---------- */

interface PidSliderProps {
    label: string;
    value: number;
    onChange: (value: number) => void;
    min: number;
    max: number;
    step: number;
}

function PidSlider({ label, value, onChange, min, max, step }: PidSliderProps) {
    return (
        <div className="group focus-within:ring-1 focus-within:ring-[#ff6b00]/30 rounded p-1 -m-1">
            <div className="flex justify-between text-[10px] mb-1 uppercase text-[#869f6a] tracking-wider">
                <span>{label}</span>
                <span className="text-[#e8f0df] font-bold tabular-nums group-focus-within:text-[#ff6b00]">
                    {value.toFixed(step < 1 ? 2 : 1)}
                </span>
            </div>
            <input
                type="range"
                className="w-full h-4 appearance-none outline-none bg-[#11160e] border border-[#2d3826] accent-[#869f6a] hover:accent-[#ff6b00] focus-visible:accent-[#ff6b00]"
                value={value}
                onChange={e => onChange(parseFloat(e.target.value))}
                min={min}
                max={max}
                step={step}
            />
        </div>
    );
}
