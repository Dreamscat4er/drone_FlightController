#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "Config.h"

/**
 * @brief Latest RC receiver state (updated by TaskRC).
 */
struct RCState {
  uint16_t ch[CHANNELS];   ///< Raw channel widths in microseconds.
  uint32_t lastUpdateUs;   ///< Timestamp of last valid frame (micros()).
  bool valid;              ///< True if data is fresh (within timeout).
};

/**
 * @brief Latest attitude state from IMU fusion (updated by TaskIMU).
 */
struct AttState {
  float angleRoll;   ///< Roll angle in degrees.
  float anglePitch;  ///< Pitch angle in degrees.
  float rateRoll;    ///< Roll rate in deg/s.
  float ratePitch;   ///< Pitch rate in deg/s.
  float rateYaw;     ///< Yaw rate in deg/s.
  uint32_t stampUs;  ///< Timestamp of measurement (micros()).
};

/**
 * @brief Single PPM pulse captured by ISR (sent via queue to TaskRC).
 */
struct PpmPulse {
  uint16_t widthUs;  ///< Pulse width in microseconds.
  bool eof;          ///< True if this pulse denotes end-of-frame (gap > PPM_EOF).
};

// === Shared RTOS primitives and states (declared here, defined in Shared.cpp) ===
extern QueueHandle_t gQPPM;           ///< Queue used by PPM ISR to send pulses to TaskRC.
extern SemaphoreHandle_t gMutexAngles;///< Mutex protecting @ref gAtt.
extern SemaphoreHandle_t gMutexRC;    ///< Mutex protecting @ref gRC.

extern RCState gRC;  ///< Global RC state (producer: TaskRC; consumers: TaskControl/TaskLog).
extern AttState gAtt;///< Global attitude state (producer: TaskIMU; consumers: TaskControl/TaskLog);

// PID state (owned by TaskControl)
extern float prevErrorRoll, prevErrorPitch, prevErrorYaw;
extern float integralRoll, integralPitch, integralYaw;
extern float prevErrorAngleRoll, prevErrorAnglePitch;
extern float integralAngleRoll, integralAnglePitch;

// PID gains (tuned in TaskControl)
extern float Kp_angle, Ki_angle, Kd_angle;
extern float Kp_rate,  Ki_rate,  Kd_rate;
extern float Kp_yaw,   Ki_yaw,   Kd_yaw;

// Gyro bias (computed and used inside TaskIMU)
extern float gyroBiasX, gyroBiasY, gyroBiasZ;

// === Task starters ===
void startTaskRC();
void startTaskIMU();
void startTaskControl();
void startTaskLog();
