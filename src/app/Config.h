/*
  Config.h
  --------
  Global configuration constants for pin mappings, task priorities,
  control limits, and timing values. Central place to tweak system settings.
*/

#pragma once
#include <Arduino.h>

// === Pin mappings ===

/** I2C SDA pin for MPU6050. */
#define SDA_PIN 5

/** I2C SCL pin for MPU6050. */
#define SCL_PIN 6

/** Input pin for PPM RC receiver signal. */
#define PPM_PIN 4

/** Motor 1 PWM pin (CW). */
#define PWM1_PIN 13
/** Motor 2 PWM pin (CCW). */
#define PWM2_PIN 10
/** Motor 3 PWM pin (CW). */
#define PWM3_PIN 11
/** Motor 4 PWM pin (CCW). */
#define PWM4_PIN 12

// === RC/PPM configuration ===

/** Number of RC channels expected in a PPM frame. */
#define CHANNELS 8

/** Gap (µs) considered end-of-frame for PPM decoding. */
#define PPM_EOF 3000

// === Motor PWM configuration ===

/** PWM frequency for ESC signals (Hz). */
#define PWM_FREQ 50

/** PWM resolution (bits) for ESP32 LEDC driver. */
#define PWM_RESOLUTION 10

/** Standard refresh period for RC/ESC signals (µs). */
#define REFRESH_PERIOD 20000

// === Control limits ===

/** Maximum commanded roll/pitch angle from RC input (degrees). */
#define MAX_ANGLE 30.0f

/** Maximum commanded yaw rate from RC input (degrees per second). */
#define MAX_YAW_RATE 75.0f

// === FreeRTOS task priorities ===

/** Barometer task priority. */
#define PRIO_BARO 2

/** Control loop task priority (highest). */
#define PRIO_CTRL   4

/** IMU task priority. */
#define PRIO_IMU    3

/** RC input task priority. */
#define PRIO_RC     2

/** Logging/debug task priority */
#define PRIO_LOG    1

// === Task periods ===

/** Barometer period in milliseconds */
#define BARO_PERIOD_MS 145

/** Control loop period in milliseconds (e.g. 5 ms = 200 Hz). */
#define CTRL_PERIOD_MS 5

/** IMU read period in milliseconds (e.g. 5 ms = 200 Hz). */
#define IMU_PERIOD_MS  5

/** Logging period in milliseconds (e.g. 100 ms = 10 Hz). */
#define LOG_PERIOD_MS  100

/**  Reference sea level pressure */
#define SEA_LEVEL_HPA 1013.25f


// === Vertical control parameters ===

/**
 * Maximum climb/descent rate command (m/s) at full stick deflection.
 * 1.5 m/s = 150 cm/s → smoother altitude response and reduced thrust spikes.
 */
#define MAX_CLIMB_RATE 1.5f

/**
 * Deadband (m/s) around zero vertical velocity.
 * If |targetVz| < VZ_DEADBAND, controller treats it as zero to prevent jitter.
 */
#define VZ_DEADBAND 0.05f

/**
 * Minimum output delta (µs) from the vertical speed PID relative to hover.
 * Negative values reduce throttle when descending or correcting overshoot.
 */
#define VZ_OUT_MIN -300

/**
 * Maximum output delta (µs) from the vertical speed PID relative to hover.
 * Positive values increase throttle when climbing or correcting undershoot.
 */
#define VZ_OUT_MAX 300

/**
 * Low-pass filter coefficient for derived vertical speed (0–1).
 * Higher values respond faster but can be noisier; lower = smoother response.
 */
#define VZ_LPF_ALPHA 0.15f