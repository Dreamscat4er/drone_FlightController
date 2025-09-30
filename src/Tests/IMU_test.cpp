/*
  imu_test.cpp
  -------------------
  IMU-only test entry point.

  This file defines its own setup()/loop() and is compiled *only*
  when the build flag IMU_ONLY_TEST is defined (env:imu_test).

  It does not start any flight tasks; it's isolated from main.cpp.
*/

#ifdef IMU_ONLY_TEST

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <app/Config.h>

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / PI)
#endif

// --- User-tunable test params ---
static constexpr float CF_ALPHA = 0.98f;     // complementary filter blend
static constexpr float GYRO_DB  = 0.5f;      // deg/s deadband
static constexpr uint32_t PRINT_MS = 50;     // print rate (ms)

// --- Globals ---
static Adafruit_MPU6050 mpu;
static float angleRoll = 0.0f, anglePitch = 0.0f;
static float biasX = 0.0f, biasY = 0.0f, biasZ = 0.0f;

static inline float deadband(float v, float th) { return (fabsf(v) < th) ? 0.0f : v; }

/** Calibrate gyro bias while stationary. */
static void calibrateGyro(size_t samples = 2000) {
  biasX = biasY = biasZ = 0.0f;
  for (size_t i = 0; i < samples; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    biasX += g.gyro.x;
    biasY += g.gyro.y;
    biasZ += g.gyro.z;
    delay(1);
  }
  biasX /= float(samples);
  biasY /= float(samples);
  biasZ /= float(samples);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== IMU Only Test (IMU_ONLY_TEST) ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!mpu.begin()) {
    Serial.println("[IMU] ERROR: MPU6050 not found!");
    while (true) delay(1000);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("[IMU] Keep board still. Calibrating gyro bias...");
  calibrateGyro();
  Serial.printf("[IMU] Bias (rad/s): bx=%.6f by=%.6f bz=%.6f\n", biasX, biasY, biasZ);

  // Seed angles from accel
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  anglePitch = atan2f(-a.acceleration.x, sqrtf(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * RAD_TO_DEG;
  angleRoll  = atan2f( a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
}

void loop() {
  static uint32_t last = 0;
  static uint32_t lastPrint = 0;

  const uint32_t periodUs = IMU_PERIOD_MS * 1000UL;
  const float dt = IMU_PERIOD_MS / 1000.0f;

  if (last == 0) last = micros();

  // Run at IMU_PERIOD_MS
  while ((micros() - last) < periodUs) {
    // spin
  }
  last += periodUs;

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  // Gyro deg/s (bias removed + deadband)
  const float gx = deadband((g.gyro.x - biasX) * RAD_TO_DEG, GYRO_DB);
  const float gy = deadband((g.gyro.y - biasY) * RAD_TO_DEG, GYRO_DB);
  const float gz = deadband((g.gyro.z - biasZ) * RAD_TO_DEG, GYRO_DB);

  // Acc angles
  const float pitchAcc = atan2f(-a.acceleration.x, sqrtf(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)) * RAD_TO_DEG;
  const float rollAcc  = atan2f( a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;

  // Complementary filter
  anglePitch = CF_ALPHA * (anglePitch + gy * dt) + (1.0f - CF_ALPHA) * pitchAcc;
  angleRoll  = CF_ALPHA * (angleRoll  + gx * dt) + (1.0f - CF_ALPHA) * rollAcc;

  // Print ~20 Hz
  const uint32_t now = millis();
  if (now - lastPrint >= PRINT_MS) {
    lastPrint = now;
    Serial.printf("t=%lu ms | Angles: R=%6.2f° P=%6.2f° | Rates: R=%6.2f P=%6.2f Y=%6.2f (deg/s)\n",
                  (unsigned long)now, angleRoll, anglePitch, gx, gy, gz);
  }
}

#endif // IMU_ONLY_TEST
