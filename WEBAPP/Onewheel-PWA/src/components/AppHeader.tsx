interface AppHeaderProps {
    title: string;
    subtitle: string;
}

export function AppHeader({ title, subtitle }: AppHeaderProps) {
    return (
        <div className="w-full max-w-sm flex flex-col items-center justify-center border-b-2 border-[#3b4731] pb-3 mb-6 relative z-10 transition-all duration-500">
            <h1 className="text-xl font-extrabold tracking-[0.2em] text-[#e8f0df] uppercase shadow-black drop-shadow-md">
                {title}
            </h1>
            <p className="text-[10px] tracking-[0.3em] text-[#ff6b00] uppercase font-bold mt-1 shadow-black drop-shadow-sm">
                {subtitle}
            </p>
        </div>
    );
}
