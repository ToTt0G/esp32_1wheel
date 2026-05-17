// ─────────────────────────────────────────────────────────────
//  BluetoothService — Singleton Web Bluetooth API wrapper
//  Manages scanning, connection, auto-reconnect, telemetry
//  subscriptions, and command dispatch.
// ─────────────────────────────────────────────────────────────

import {
    SERVICE_UUID,
    TELEMETRY_CHAR_UUID,
    CONTROL_CHAR_UUID,
    DEVICE_INFO_CHAR_UUID,
    CONFIG_CHAR_UUID,
    parseTelemetry,
    parseDeviceInfo,
    parseConfig,
    buildCalibrateCommand,
    type TelemetryPacket,
    type DeviceInfo,
    type ConfigPacket,
} from './ble-protocol';

// ── Types ───────────────────────────────────────────────────

export type BleConnectionState = 'disconnected' | 'scanning' | 'connecting' | 'connected';

type StateChangeCallback = (state: BleConnectionState) => void;
type TelemetryCallback = (data: TelemetryPacket) => void;
type ConfigCallback = (config: ConfigPacket) => void;
type LogCallback = (message: string) => void;
type ErrorCallback = (error: string) => void;

// ── Constants ───────────────────────────────────────────────

const MAX_RECONNECT_ATTEMPTS = 10;
const INITIAL_RECONNECT_DELAY_MS = 1000;
const MAX_RECONNECT_DELAY_MS = 30000;

// ── Service ─────────────────────────────────────────────────

class BluetoothService {
    // Connection state
    private _state: BleConnectionState = 'disconnected';
    private _device: BluetoothDevice | null = null;
    private _server: BluetoothRemoteGATTServer | null = null;

    // Characteristics
    private _telemetryChar: BluetoothRemoteGATTCharacteristic | null = null;
    private _controlChar: BluetoothRemoteGATTCharacteristic | null = null;
    private _deviceInfoChar: BluetoothRemoteGATTCharacteristic | null = null;
    private _configChar: BluetoothRemoteGATTCharacteristic | null = null;

    // Auto-reconnect
    private _shouldReconnect = false;
    private _reconnectAttempt = 0;
    private _reconnectTimer: ReturnType<typeof setTimeout> | null = null;

    // Device info cache
    private _deviceInfo: DeviceInfo | null = null;
    private _deviceConfig: ConfigPacket | null = null;

    // Callbacks
    private _stateCallbacks = new Set<StateChangeCallback>();
    private _telemetryCallbacks = new Set<TelemetryCallback>();
    private _configCallbacks = new Set<ConfigCallback>();
    private _logCallbacks = new Set<LogCallback>();
    private _errorCallbacks = new Set<ErrorCallback>();

    // Bound handler reference (so we can remove it)
    private _boundDisconnectHandler: (() => void) | null = null;
    private _boundTelemetryHandler: ((event: Event) => void) | null = null;

    // ── Public API ──────────────────────────────────────────

    /** Check if Web Bluetooth is available */
    get isSupported(): boolean {
        return typeof navigator !== 'undefined' && 'bluetooth' in navigator;
    }

    get state(): BleConnectionState {
        return this._state;
    }

    get deviceInfo(): DeviceInfo | null {
        return this._deviceInfo;
    }

    get deviceConfig(): ConfigPacket | null {
        return this._deviceConfig;
    }

    /**
     * Scan for a device and connect.
     * This triggers the native browser device picker (requires user gesture).
     */
    async scan(): Promise<void> {
        if (!this.isSupported) {
            this._emitError('Web Bluetooth is not supported in this browser.');
            return;
        }

        if (this._state === 'connecting' || this._state === 'connected') {
            this._emitLog('[BLE] Already connected or connecting.');
            return;
        }

        try {
            this._setState('scanning');
            this._emitLog('[BLE] Requesting device...');

            // The browser will show its native device picker here
            const device = await navigator.bluetooth.requestDevice({
                filters: [{ services: [SERVICE_UUID] }],
                optionalServices: [SERVICE_UUID],
            });

            this._device = device;
            this._emitLog(`[BLE] Selected: ${device.name ?? 'Unknown Device'}`);

            await this._connectToDevice(device);
        } catch (err) {
            if (err instanceof DOMException && err.name === 'NotFoundError') {
                // User cancelled the picker
                this._emitLog('[BLE] Scan cancelled by user.');
                this._setState('disconnected');
            } else {
                const msg = err instanceof Error ? err.message : String(err);
                this._emitError(`[BLE] Scan failed: ${msg}`);
                this._setState('disconnected');
            }
        }
    }

    /** Disconnect from the current device */
    disconnect(): void {
        this._shouldReconnect = false;
        this._cancelReconnect();
        this._cleanup();
        this._setState('disconnected');
        this._emitLog('[BLE] Connection Terminated.');
    }

    /** Send a binary command to the control characteristic */
    async sendCommand(buffer: ArrayBuffer): Promise<void> {
        if (!this._controlChar) {
            this._emitError('[BLE] Not connected — cannot send command.');
            return;
        }

        try {
            await this._controlChar.writeValueWithoutResponse(new Uint8Array(buffer));
            this._emitLog(`[BLE] CMD sent (${buffer.byteLength} bytes)`);
        } catch (err) {
            const msg = err instanceof Error ? err.message : String(err);
            this._emitError(`[BLE] Write failed: ${msg}`);
        }
    }

    /** Send calibration command to zero the sensor */
    async calibrate(): Promise<void> {
        this._emitLog('[BLE] Requesting sensor calibration...');
        await this.sendCommand(buildCalibrateCommand());
    }

    // ── Event subscription ──────────────────────────────────

    onStateChange(cb: StateChangeCallback): () => void {
        this._stateCallbacks.add(cb);
        return () => this._stateCallbacks.delete(cb);
    }

    onTelemetry(cb: TelemetryCallback): () => void {
        this._telemetryCallbacks.add(cb);
        return () => this._telemetryCallbacks.delete(cb);
    }

    onConfigChange(cb: ConfigCallback): () => void {
        this._configCallbacks.add(cb);
        return () => this._configCallbacks.delete(cb);
    }

    onLog(cb: LogCallback): () => void {
        this._logCallbacks.add(cb);
        return () => this._logCallbacks.delete(cb);
    }

    onError(cb: ErrorCallback): () => void {
        this._errorCallbacks.add(cb);
        return () => this._errorCallbacks.delete(cb);
    }

    // ── Private: Connection ─────────────────────────────────

    private async _connectToDevice(device: BluetoothDevice): Promise<void> {
        this._setState('connecting');
        this._emitLog(`[BLE] Connecting to ${device.name ?? device.id}...`);

        try {
            // Listen for disconnects
            this._boundDisconnectHandler = () => this._handleDisconnect();
            device.addEventListener('gattserverdisconnected', this._boundDisconnectHandler);

            // Connect to GATT server
            const server = await device.gatt!.connect();
            this._server = server;
            this._emitLog('[BLE] GATT server connected.');

            // Get our service
            const service = await server.getPrimaryService(SERVICE_UUID);
            this._emitLog('[BLE] Service discovered.');

            // Get characteristics
            this._telemetryChar = await service.getCharacteristic(TELEMETRY_CHAR_UUID);
            this._controlChar = await service.getCharacteristic(CONTROL_CHAR_UUID);

            // Device info is optional — don't fail if missing
            try {
                this._deviceInfoChar = await service.getCharacteristic(DEVICE_INFO_CHAR_UUID);
                const infoValue = await this._deviceInfoChar.readValue();
                this._deviceInfo = parseDeviceInfo(infoValue);
                this._emitLog(`[BLE] Device: ${this._deviceInfo.name} (FW ${this._deviceInfo.firmware})`);
            } catch {
                this._emitLog('[BLE] Device info characteristic not available.');
                this._deviceInfo = null;
            }

            // Sync Config (PID/Threshold)
            try {
                this._configChar = await service.getCharacteristic(CONFIG_CHAR_UUID);
                const configValue = await this._configChar.readValue();
                this._deviceConfig = parseConfig(configValue);
                this._emitLog(`[BLE] Config synced: P=${this._deviceConfig.p} I=${this._deviceConfig.i} D=${this._deviceConfig.d} Thr=${this._deviceConfig.fpThr}`);
                this._configCallbacks.forEach(cb => cb(this._deviceConfig!));
            } catch (err) {
                this._emitLog('[BLE] Config characteristic not available (auto-sync disabled).');
                this._deviceConfig = null;
            }

            // Start telemetry notifications
            await this._startTelemetry();

            // Connected!
            this._shouldReconnect = true;
            this._reconnectAttempt = 0;
            this._setState('connected');
            this._emitLog('[BLE] SECURE CONNECTION ESTABLISHED');
        } catch (err) {
            const msg = err instanceof Error ? err.message : String(err);
            this._emitError(`[BLE] Connection failed: ${msg}`);
            this._cleanup();
            this._setState('disconnected');
        }
    }

    private async _startTelemetry(): Promise<void> {
        if (!this._telemetryChar) return;

        this._boundTelemetryHandler = (event: Event) => {
            const characteristic = event.target as BluetoothRemoteGATTCharacteristic;
            const value = characteristic.value;
            if (!value) return;

            try {
                const packet = parseTelemetry(value);
                this._telemetryCallbacks.forEach(cb => cb(packet));
            } catch (err) {
                console.warn('[BLE] Telemetry parse error:', err);
            }
        };

        await this._telemetryChar.startNotifications();
        this._telemetryChar.addEventListener('characteristicvaluechanged', this._boundTelemetryHandler);
        this._emitLog('[BLE] Telemetry stream active (20 Hz).');
    }

    // ── Private: Disconnect & Reconnect ─────────────────────

    private _handleDisconnect(): void {
        this._emitLog('[BLE] Connection lost.');
        this._setState('disconnected');

        // Clean up characteristic refs (they're invalid after disconnect)
        this._telemetryChar = null;
        this._controlChar = null;
        this._deviceInfoChar = null;
        this._configChar = null;
        this._server = null;

        if (this._shouldReconnect && this._device) {
            this._attemptReconnect();
        }
    }

    private _attemptReconnect(): void {
        if (this._reconnectAttempt >= MAX_RECONNECT_ATTEMPTS) {
            this._emitError(`[BLE] Reconnect failed after ${MAX_RECONNECT_ATTEMPTS} attempts.`);
            this._shouldReconnect = false;
            return;
        }

        const delay = Math.min(
            INITIAL_RECONNECT_DELAY_MS * Math.pow(2, this._reconnectAttempt),
            MAX_RECONNECT_DELAY_MS
        );

        this._reconnectAttempt++;
        this._emitLog(`[BLE] Reconnecting in ${(delay / 1000).toFixed(1)}s (attempt ${this._reconnectAttempt}/${MAX_RECONNECT_ATTEMPTS})...`);

        this._reconnectTimer = setTimeout(async () => {
            if (!this._device || !this._shouldReconnect) return;

            try {
                await this._connectToDevice(this._device);
            } catch {
                this._attemptReconnect();
            }
        }, delay);
    }

    private _cancelReconnect(): void {
        if (this._reconnectTimer) {
            clearTimeout(this._reconnectTimer);
            this._reconnectTimer = null;
        }
        this._reconnectAttempt = 0;
    }

    // ── Private: Cleanup ────────────────────────────────────

    private _cleanup(): void {
        // Stop telemetry notifications
        if (this._telemetryChar && this._boundTelemetryHandler) {
            try {
                this._telemetryChar.removeEventListener('characteristicvaluechanged', this._boundTelemetryHandler);
                this._telemetryChar.stopNotifications().catch(() => { });
            } catch { /* already disconnected */ }
        }

        // Remove disconnect listener
        if (this._device && this._boundDisconnectHandler) {
            this._device.removeEventListener('gattserverdisconnected', this._boundDisconnectHandler);
        }

        // Disconnect GATT
        if (this._server?.connected) {
            try {
                this._server.disconnect();
            } catch { /* already disconnected */ }
        }

        // Reset refs
        this._telemetryChar = null;
        this._controlChar = null;
        this._deviceInfoChar = null;
        this._configChar = null;
        this._server = null;
        this._boundDisconnectHandler = null;
        this._boundTelemetryHandler = null;
    }

    // ── Private: State & Events ─────────────────────────────

    private _setState(state: BleConnectionState): void {
        this._state = state;
        this._stateCallbacks.forEach(cb => cb(state));
    }

    private _emitLog(message: string): void {
        console.log(message);
        this._logCallbacks.forEach(cb => cb(message));
    }

    private _emitError(message: string): void {
        console.error(message);
        this._errorCallbacks.forEach(cb => cb(message));
        this._logCallbacks.forEach(cb => cb(message));
    }
}

// ── Singleton Export ─────────────────────────────────────────

export const bluetoothService = new BluetoothService();
