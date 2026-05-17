import { useState, useEffect, useCallback } from 'react';

interface BeforeInstallPromptEvent extends Event {
    prompt: () => Promise<void>;
    userChoice: Promise<{ outcome: 'accepted' | 'dismissed' }>;
}

let cachedPromptEvent: BeforeInstallPromptEvent | null = null;

// Catch the event as early as possible before React mounts
window.addEventListener('beforeinstallprompt', (e) => {
    e.preventDefault();
    cachedPromptEvent = e as BeforeInstallPromptEvent;
    // Dispatch a custom event in case our hook is already mounted
    window.dispatchEvent(new Event('deferredpromptcached'));
});

export function useInstallPrompt() {
    const [deferredPrompt, setDeferredPrompt] = useState<BeforeInstallPromptEvent | null>(cachedPromptEvent);
    const [showInstallPrompt, setShowInstallPrompt] = useState(!!cachedPromptEvent);

    useEffect(() => {
        // Handle events that fire after component mount
        const handleBeforeInstallPrompt = (e: Event) => {
            e.preventDefault();
            cachedPromptEvent = e as BeforeInstallPromptEvent;
            setDeferredPrompt(cachedPromptEvent);
            setShowInstallPrompt(true);
        };

        // Handle the custom event if the original event fired but React state isn't updated
        const handleCachedPrompt = () => {
            if (cachedPromptEvent) {
                setDeferredPrompt(cachedPromptEvent);
                setShowInstallPrompt(true);
            }
        };

        const handleAppInstalled = () => {
            setShowInstallPrompt(false);
            setDeferredPrompt(null);
            cachedPromptEvent = null;
            console.log('PWA was installed');
        };

        window.addEventListener('beforeinstallprompt', handleBeforeInstallPrompt);
        window.addEventListener('deferredpromptcached', handleCachedPrompt);
        window.addEventListener('appinstalled', handleAppInstalled);

        // Check standalone and handle edge cases
        const isStandalone = window.matchMedia('(display-mode: standalone)').matches;
        if (isStandalone) {
            setShowInstallPrompt(false);
        } else if (cachedPromptEvent) {
            setShowInstallPrompt(true);
        }

        return () => {
            window.removeEventListener('beforeinstallprompt', handleBeforeInstallPrompt);
            window.removeEventListener('deferredpromptcached', handleCachedPrompt);
            window.removeEventListener('appinstalled', handleAppInstalled);
        };
    }, []);

    const install = useCallback(async () => {
        if (!deferredPrompt) return;

        deferredPrompt.prompt();
        const { outcome } = await deferredPrompt.userChoice;
        console.log(`User response to the install prompt: ${outcome}`);

        setDeferredPrompt(null);
        setShowInstallPrompt(false);
    }, [deferredPrompt]);

    const dismiss = useCallback(() => {
        setShowInstallPrompt(false);
    }, []);

    return { showInstallPrompt, install, dismiss };
}
