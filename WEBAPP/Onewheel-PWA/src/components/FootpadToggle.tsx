interface FootpadToggleProps {
    isArmed: boolean;
    onToggle: () => void;
}

export function FootpadToggle({ isArmed, onToggle }: FootpadToggleProps) {
    return (
        <button
            onClick={onToggle}
            className={`border p-2 flex flex-col items-center justify-center transition-all duration-200 outline-none focus-visible:ring-2 focus-visible:ring-offset-2 focus-visible:ring-offset-[#1c2317] focus-visible:ring-[#ff6b00] ${isArmed
                    ? 'bg-[#ff6b00] border-[#ff6b00] text-[#11160e] shadow-[0_0_15px_rgba(255,107,0,0.4)]'
                    : 'bg-[#11160e] border-[#3b4731] hover:bg-[#181d14]'
                }`}
        >
            <div className={`text-[9px] uppercase tracking-widest mb-2 ${isArmed ? 'text-[#11160e]/80' : 'text-[#869f6a]'}`}>
                Deck Array
            </div>
            <div className="w-12 h-12 border-2 flex items-center justify-center relative border-current">
                {isArmed ? (
                    <div className="w-8 h-8 bg-[#11160e]" />
                ) : (
                    <div className="w-8 h-8 border border-dashed border-[#869f6a]/50" />
                )}
            </div>
            <div className="text-[10px] font-bold mt-2 tracking-wider">
                {isArmed ? 'ARMED' : 'SAFE'}
            </div>
        </button>
    );
}
