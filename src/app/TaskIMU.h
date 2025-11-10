#pragma once
#include "Shared.h"

/**
 * @brief Start the IMU task (MPU6050 read + complementary filter).
 *
 * - Initializes I2C and the MPU6050.
 * - Calibrates gyro bias at boot.
 * - Runs at IMU_PERIOD_MS, publishing angles/rates to @ref gAtt.
 * - Optionally applies a gentle low-pass filter (LPF) to the World Z acceleration.
 *
 * Thread safety via @ref gMutexAngles.
 */
void startTaskIMU();
