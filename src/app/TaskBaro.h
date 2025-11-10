#pragma once
/**
 * @brief Start the barometer task (BMP280 altitude sensing).
 *
 * - Initializes the BMP280 sensor on the shared I²C bus.
 * - Performs altitude-zero calibration at startup by averaging N samples.
 * - Runs periodically at @ref BARO_PERIOD_MS.
 * - Reads temperature, pressure, and altitude each cycle.
 * - Optionally applies a gentle low-pass filter (LPF) to the zeroed altitude.
 * - Publishes @ref gBaro for use by the control loop.
 * 
 * -Thread safery via @ref gMutexBaro.
 */
void startTaskBaro();
