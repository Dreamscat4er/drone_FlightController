#pragma once
#include <Arduino.h>
#include "Config.h"

/**
 * @brief Apply a deadband to filter out small values (e.g., sensor noise).
 *
 * Values with |value| < threshold are returned as 0; others pass through unchanged.
 *
 * @param value     Input value (e.g., gyro rate in deg/s).
 * @param threshold Absolute threshold for deadband (same units as @p value).
 * @return Filtered value after deadband.
 */
float applyDeadband(float value, float threshold);

/**
 * @brief Constrain a value and reset integral term on saturation (anti-windup).
 *
 * If @p value exceeds [minVal, maxVal], the referenced @p integral is zeroed to
 * prevent integral windup in controllers.
 *
 * @param value     Value to constrain.
 * @param integral  Reference to integrator accumulator (may be modified).
 * @param minVal    Lower output limit.
 * @param maxVal    Upper output limit.
 * @return Constrained value within [minVal, maxVal].
 */
float constrainAndAntiWindup(float value, float &integral, float minVal, float maxVal);

/**
 * @brief Generic PID controller with error deadband and anti-windup.
 *
 * Computes: out = Kp*e + Ki*∫e dt + Kd*de/dt, then constrains to [outMin, outMax].
 * If |e| < errDB, e is treated as 0; integral is slightly bled when e==0.
 *
 * @param target      Desired setpoint.
 * @param current     Measured value.
 * @param integral    Reference to integrator accumulator (updated in-place).
 * @param prev_error  Reference to previous cycle error (updated in-place).
 * @param Kp          Proportional gain.
 * @param Ki          Integral gain.
 * @param Kd          Derivative gain.
 * @param dt          Controller timestep in seconds.
 * @param outMin      Minimum output limit.
 * @param outMax      Maximum output limit.
 * @param errDB       Error deadband threshold.
 * @return Controller output after constraints.
 */
float computePID(float target, float current, float &integral, float &prev_error,
                 float Kp, float Ki, float Kd, float dt,
                 float outMin = -500, float outMax = 500, float errDB = 2);

/**
 * @brief Convert a servo-style pulse width in microseconds to ESP32 LEDC ticks.
 *
 * Maps 1000–2000 µs into the configured LEDC resolution over a 20 ms period.
 *
 * @param usWidth Pulse width in microseconds (typically 1000–2000).
 * @return LEDC duty tick value for ledcWrite().
 */
uint16_t usToTicks(uint16_t usWidth);
