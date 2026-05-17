import { useEffect, useRef } from 'react';

/**
 * Screen Wake Lock — prevents the phone screen from turning off.
 * Critical for safety: you never want the screen to auto-lock
 * while checking speed/battery on a onewheel.
 *
 * Automatically acquires the lock when `enabled` is true,
 * re-acquires on page visibility change, and releases on
 * disable or unmount.
 */
export function useWakeLock(enabled: boolean): void {
    const wakeLockRef = useRef<WakeLockSentinel | null>(null);

    useEffect(() => {
        if (!enabled || !('wakeLock' in navigator)) return;

        let released = false;

        const acquire = async () => {
            if (released) return;
            try {
                wakeLockRef.current = await navigator.wakeLock.request('screen');
                wakeLockRef.current.addEventListener('release', () => {
                    wakeLockRef.current = null;
                });
            } catch (err) {
                console.warn('[WakeLock] Failed to acquire:', err);
            }
        };

        // Re-acquire when tab becomes visible again
        const handleVisibility = () => {
            if (document.visibilityState === 'visible' && !released) {
                acquire();
            }
        };

        acquire();
        document.addEventListener('visibilitychange', handleVisibility);

        return () => {
            released = true;
            document.removeEventListener('visibilitychange', handleVisibility);
            if (wakeLockRef.current) {
                wakeLockRef.current.release().catch(() => { });
                wakeLockRef.current = null;
            }
        };
    }, [enabled]);
}
