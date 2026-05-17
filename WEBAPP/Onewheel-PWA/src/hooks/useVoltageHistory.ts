import { useState, useEffect, useRef } from 'react';

const HISTORY_LENGTH = 60;

/**
 * Maintains a rolling voltage history buffer for the oscilloscope.
 * Only records when `active` is true (i.e. BLE connected).
 */
export function useVoltageHistory(voltage: number, active: boolean): number[] {
    const [history, setHistory] = useState<number[]>(() => Array(HISTORY_LENGTH).fill(0));
    const prevVoltage = useRef(voltage);

    useEffect(() => {
        if (active && voltage !== prevVoltage.current) {
            // eslint-disable-next-line react-hooks/set-state-in-effect
            setHistory(prev => [...prev.slice(1), voltage]);
            prevVoltage.current = voltage;
        }
    }, [voltage, active]);

    return history;
}
