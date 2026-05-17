// Wake Lock API — not yet in standard TypeScript lib types
// See: https://developer.mozilla.org/en-US/docs/Web/API/Screen_Wake_Lock_API

interface WakeLockSentinel extends EventTarget {
    readonly released: boolean;
    readonly type: 'screen';
    release(): Promise<void>;
    onrelease: ((this: WakeLockSentinel, ev: Event) => void) | null;
}

interface WakeLock {
    request(type: 'screen'): Promise<WakeLockSentinel>;
}

interface Navigator {
    readonly wakeLock: WakeLock;
}
