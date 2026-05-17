import { useMemo } from 'react';

interface OscilloscopeProps {
    history: number[];
}

export function Oscilloscope({ history }: OscilloscopeProps) {
    const pathData = useMemo(() => generatePath(history), [history]);

    return (
        <div className="bg-[#11160e] border border-[#3b4731] p-2 flex flex-col relative overflow-hidden group">
            <div className="text-[9px] text-[#869f6a] uppercase tracking-widest mb-1 z-10 flex justify-between">
                <span>V-CORE Scope</span>
                <span className="text-[#ff6b00] opacity-0 group-hover:opacity-100 transition-opacity">REC</span>
            </div>
            <div className="flex-1 relative w-full h-16 border border-[#2d3826] bg-[#0a0d08]">
                {/* Grid overlay */}
                <div className="absolute inset-0 bg-[linear-gradient(rgba(134,159,106,0.1)_1px,transparent_1px),linear-gradient(90deg,rgba(134,159,106,0.1)_1px,transparent_1px)] bg-[size:10px_10px] pointer-events-none" />
                <svg viewBox="0 0 100 30" className="w-full h-full preserve-aspect-ratio-none">
                    <path d={pathData} fill="none" stroke="#ff6b00" strokeWidth="0.8" vectorEffect="non-scaling-stroke" strokeLinejoin="round" />
                </svg>
                {/* Scanline overlay */}
                <div className="absolute inset-0 bg-[linear-gradient(transparent_50%,rgba(0,0,0,0.5)_50%)] bg-[length:100%_4px] pointer-events-none opacity-50" />
            </div>
        </div>
    );
}

function generatePath(history: number[]): string {
    const w = 100;
    const h = 30;
    const minH = Math.min(...history) - 0.5;
    const maxH = Math.max(...history) + 0.5;
    const range = maxH - minH || 1;

    return 'M ' + history.map((v, i) => {
        const x = (i / (history.length - 1)) * w;
        const y = h - ((v - minH) / range) * h;
        return `${x},${y}`;
    }).join(' L ');
}
