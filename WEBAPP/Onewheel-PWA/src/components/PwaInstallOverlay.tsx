interface PwaInstallOverlayProps {
    onInstall: () => void;
    onDismiss: () => void;
}

export function PwaInstallOverlay({ onInstall, onDismiss }: PwaInstallOverlayProps) {
    return (
        <div className="absolute inset-x-0 bottom-0 z-50 p-4 sm:p-6 pb-8 animate-in slide-in-from-bottom duration-500">
            <div className="w-full max-w-sm mx-auto bg-[#11160e] rounded-sm shadow-[0_-5px_30px_rgba(0,0,0,0.8)] p-6 border-t-4 border-[#ff6b00] border-l border-r border-[#3b4731] relative flex flex-col items-center text-center">
                {/* Close button */}
                <button
                    onClick={onDismiss}
                    className="absolute top-2 right-2 w-8 h-8 flex items-center justify-center text-[#869f6a] hover:text-white"
                >
                    <svg xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24" strokeWidth={2} stroke="currentColor" className="w-5 h-5">
                        <path strokeLinecap="round" strokeLinejoin="round" d="M6 18L18 6M6 6l12 12" />
                    </svg>
                </button>

                {/* Icon */}
                <div className="w-16 h-16 rounded-full bg-[#1c2317] border-2 border-[#ff6b00] flex items-center justify-center mb-4 shadow-[0_0_15px_rgba(255,107,0,0.3)]">
                    <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 24 24" fill="currentColor" className="w-8 h-8 text-[#ff6b00]">
                        <path d="M11.47 3.84a.75.75 0 011.06 0l8.69 8.69a.75.75 0 101.06-1.06l-8.689-8.69a2.25 2.25 0 00-3.182 0l-8.69 8.69a.75.75 0 001.06 1.06l8.69-8.69z" />
                        <path d="M12 5.432l8.159 8.159c.03.03.06.058.091.086v6.198c0 1.035-.84 1.875-1.875 1.875H15a.75.75 0 01-.75-.75v-4.5a.75.75 0 00-.75-.75h-3a.75.75 0 00-.75.75V21a.75.75 0 01-.75.75H5.625a1.875 1.875 0 01-1.875-1.875v-6.198a2.29 2.29 0 00.091-.086L12 5.43z" />
                    </svg>
                </div>

                <h3 className="text-xl font-bold tracking-widest text-[#e8f0df] mb-2 uppercase">SYSTEM DEPLOYMENT</h3>
                <p className="text-xs text-[#869f6a] mb-6 tracking-wide">
                    Install DIY ONEWHEEL directly to your home screen for persistent telemetry access and offline mode.
                </p>

                <button
                    onClick={onInstall}
                    className="w-full py-3 bg-[#ff6b00] text-[#11160e] font-bold uppercase tracking-widest text-sm hover:bg-white transition-colors shadow-[0_0_10px_rgba(255,107,0,0.4)] hover:shadow-[0_0_15px_rgba(255,255,255,0.6)]"
                >
                    INSTALL APP
                </button>
            </div>
        </div>
    );
}
