import { useState, useEffect } from 'react';

export function useGeolocation() {
    const isSupported = 'geolocation' in navigator;
    const [speedKmh, setSpeedKmh] = useState<number>(0);
    const [error, setError] = useState<string | null>(isSupported ? null : 'Geolocation is not supported by your browser');
    const [hasPermission, setHasPermission] = useState<boolean | null>(isSupported ? null : false);

    useEffect(() => {
        if (!isSupported) {
            return;
        }

        const watchId = navigator.geolocation.watchPosition(
            (position) => {
                setHasPermission(true);
                setError(null);

                // position.coords.speed is in meters per second (m/s)
                // Multiply by 3.6 to convert to kilometers per hour (km/h)
                const speedMs = position.coords.speed;
                if (speedMs !== null && speedMs >= 0) {
                    setSpeedKmh(speedMs * 3.6);
                } else {
                    // Sometimes speed is null on desktop or when not moving enough
                    setSpeedKmh(0);
                }
            },
            (err) => {
                setError(err.message);
                if (err.code === err.PERMISSION_DENIED) {
                    setHasPermission(false);
                }
                setSpeedKmh(0);
            },
            {
                enableHighAccuracy: true,
                maximumAge: 0,
                timeout: 5000
            }
        );

        return () => {
            navigator.geolocation.clearWatch(watchId);
        };
    }, []);

    return { speedKmh, error, hasPermission };
}
