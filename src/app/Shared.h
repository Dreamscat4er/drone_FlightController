#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "Config.h"

/*
 * Project-wide conventions
 * ------------------------
 *  Angles: degrees; angular rates: deg/s
 *  Acceleration: m/s^2 (g = 9.80665)
 *  Altitude (z): meters, +up; Vertical velocity (Vz): m/s, +up
 *  Timestamps: micros()
 */

// === RC input ===============================================================
struct RCState {
  uint16_t ch[CHANNELS];    ///< Raw channel widths (µs)
  uint32_t lastUpdateUs;    ///< Time of last valid frame (micros())
  bool     valid;           ///< True if fresh within timeout
};

// === Motor outputs ==========================================================
struct MotorFeed {
  int m1;              ///< ESC pulse (µs)
  int m2;              ///< ESC pulse (µs)
  int m3;              ///< ESC pulse (µs)
  int m4;              ///< ESC pulse (µs)
};

// === Attitude from IMU fusion ==============================================
struct AttState {
  float     angleRoll;      ///< Roll [deg]
  float     anglePitch;     ///< Pitch [deg]
  float     rateRoll;       ///< Roll rate [deg/s]
  float     ratePitch;      ///< Pitch rate [deg/s]
  float     rateYaw;        ///< Yaw rate [deg/s]
  uint32_t  stampUs;        ///< Timestamp [micros()]
};

// === Barometer ==============================================================
/*
 * For the EKF only altitude needed as a measurement (no baro velocity here).
 */
struct BaroState {
  float     pressure;       ///< Pa
  float     temperature;    ///< °C
  float     altitude;       ///< m (+up), pressure-derived (relative or QNH-adjusted)
  uint32_t  stampUs;        ///< Timestamp [micros()]
  bool      valid;          ///< True if fresh within timeout
};

// === Vertical acceleration (world frame, gravity removed) ==================
/*
 * Published by TaskIMU after rotating body accel to world frame and removing g.
 * This is the EKF process input.
 */
struct VertAccState {
  float     aZlin;          ///< m/s^2 (+up), gravity removed, world frame
  uint32_t  stampUs;        ///< Timestamp [micros()]
  bool      valid;          ///< True if recent/usable
};

// === PPM pulse from ISR =====================================================
struct PpmPulse {
  uint16_t widthUs;         ///< Pulse width (µs)
  bool     eof;             ///< End-of-frame (gap > PPM_EOF)
};

// === Shared RTOS primitives and states
extern QueueHandle_t      gQPPM;            ///< PPM pulse queue → TaskRC
extern SemaphoreHandle_t  gMutexAngles;     ///< Protects gAtt
extern SemaphoreHandle_t  gMutexRC;         ///< Protects gRC
extern SemaphoreHandle_t  gMutexMotors;     ///< Protects gMotors
extern SemaphoreHandle_t  gMutexBaro;       ///< Protects gBaro
extern SemaphoreHandle_t  gMutexI2C;        ///< I2C bus lock (IMU + Baro share)
extern SemaphoreHandle_t  gMutexVertAcc;    ///< Protects gVertAcc

extern RCState        gRC;       ///< Producer: TaskRC
extern AttState       gAtt;      ///< Producer: TaskIMU
extern MotorFeed      gMotors;   ///< Producer: TaskControl
extern BaroState      gBaro;     ///< Producer: TaskBaro
extern VertAccState   gVertAcc;  ///< Producer: TaskIMU

// === PID state (owned by TaskControl) ======================================
// Angle/Rate loops
extern float prevErrorRoll, prevErrorPitch, prevErrorYaw;
extern float integralRoll,  integralPitch,  integralYaw;
extern float prevErrorAngleRoll, prevErrorAnglePitch;
extern float integralAngleRoll,  integralAnglePitch;

// Vertical rate loop (Vz)
extern float prevErrorVz, integralVz;

// === PID gains (tuned in TaskControl) ======================================
extern float Kp_angle, Ki_angle, Kd_angle;
extern float Kp_rate,  Ki_rate,  Kd_rate;
extern float Kp_yaw,   Ki_yaw,   Kd_yaw;
extern float Kp_alt,   Ki_alt,   Kd_alt;
extern float Kp_vz,    Ki_vz,    Kd_vz;


// === Task starters ==========================================================
void startTaskRC();
void startTaskIMU();
void startTaskControl();
void startTaskLog();
void startTaskBaro();

