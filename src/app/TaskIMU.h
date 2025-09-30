#pragma once
#include "Shared.h"

/**
 * @brief Start the IMU task (MPU6050 read + complementary filter).
 *
 * - Initializes I2C and the MPU6050.
 * - Calibrates gyro bias at boot (keep craft still).
 * - Runs at IMU_PERIOD_MS, publishing angles/rates to @ref gAtt.
 *
 * Thread safety via @ref gMutexAngles.
 */
void startTaskIMU();
