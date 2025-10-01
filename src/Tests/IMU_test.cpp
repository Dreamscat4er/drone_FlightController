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

// ---------- User-tunable test params ----------
static constexpr float CF_ALPHA  = 0.98f;    // complementary filter blend
static constexpr float GYRO_DB   = 0.5f;     // deg/s deadband
static constexpr uint32_t PRINT_MS = 50;     // print rate (ms)

// ---------- Mounting configuration ----------
// IMU is upside-down. Choose which 180° axis matches your physical mount.
// Set exactly one of these to true.
static constexpr bool MOUNT_RX_180 = true;   // 180° about X (roll)  ← default
static constexpr bool MOUNT_RY_180 = false;  // 180° about Y (pitch)
static constexpr bool MOUNT_RZ_180 = false;  // 180° about Z (yaw)

// ---------- Globals ----------
static Adafruit_MPU6050 mpu;
static float angleRoll = 0.0f, anglePitch = 0.0f;
static float biasX = 0.0f, biasY = 0.0f, biasZ = 0.0f;

// ---------- Helpers ----------
static inline float deadband(float v, float th) { return (fabsf(v) < th) ? 0.0f : v; }

static inline float wrap180(float a) {
  while (a > 180.f)  a -= 360.f;
  while (a <= -180.f) a += 360.f;
  return a;
}

// Apply fixed mounting rotation (180°) to accel and gyro (deg/s) vectors
static inline void applyMountAccel(float& ax, float& ay, float& az) {
  if (MOUNT_RX_180) { ay = -ay; az = -az; }
  if (MOUNT_RY_180) { ax = -ax; az = -az; }
  if (MOUNT_RZ_180) { ax = -ax; ay = -ay; }
}
static inline void applyMountGyro(float& gx, float& gy, float& gz) {
  if (MOUNT_RX_180) { gy = -gy; gz = -gz; }
  if (MOUNT_RY_180) { gx = -gx; gz = -gz; }
  if (MOUNT_RZ_180) { gx = -gx; gy = -gy; }
}

/** Calibrate gyro bias while stationary (sensor-frame). */
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

  // Seed angles from accel (transform first into body frame)
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  applyMountAccel(ax, ay, az);

  anglePitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;
  angleRoll  = atan2f( ay,  az) * RAD_TO_DEG;

  anglePitch = wrap180(anglePitch);
  angleRoll  = wrap180(angleRoll);
}

void loop() {
  static uint32_t last = 0;
  static uint32_t lastPrint = 0;

  const uint32_t periodUs = IMU_PERIOD_MS * 1000UL;
  const float dt = IMU_PERIOD_MS / 1000.0f;

  if (last == 0) last = micros();

  // Run at IMU_PERIOD_MS
  while ((micros() - last) < periodUs) {
    // spin-wait
  }
  last += periodUs;

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  // 1) Accelerometer (transform to body frame)
  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  applyMountAccel(ax, ay, az);

  // 2) Gyro: bias removal (sensor-frame), deadband, then transform to body frame (deg/s)
  float gx_s = deadband((g.gyro.x - biasX) * RAD_TO_DEG, GYRO_DB);
  float gy_s = deadband((g.gyro.y - biasY) * RAD_TO_DEG, GYRO_DB);
  float gz_s = deadband((g.gyro.z - biasZ) * RAD_TO_DEG, GYRO_DB);

  float gx = gx_s, gy = gy_s, gz = gz_s;
  applyMountGyro(gx, gy, gz);

  // 3) Accel-derived angles (body frame)
  const float pitchAcc = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;
  const float rollAcc  = atan2f( ay,  az) * RAD_TO_DEG;

  // 4) Seam-safe complementary filter (blend wrapped error)
  angleRoll  = wrap180(angleRoll  + gx * dt);
  anglePitch = wrap180(anglePitch + gy * dt);

  const float errRoll  = wrap180(rollAcc  - angleRoll);
  const float errPitch = wrap180(pitchAcc - anglePitch);

  angleRoll  = wrap180(angleRoll  + (1.0f - CF_ALPHA) * errRoll);
  anglePitch = wrap180(anglePitch + (1.0f - CF_ALPHA) * errPitch);

  // 5) Print (body-frame angles and rates)
  const uint32_t now = millis();
  if (now - lastPrint >= PRINT_MS) {
    lastPrint = now;
    Serial.printf("t=%lu ms | Angles: R=%6.2f° P=%6.2f° | Rates: R=%6.2f P=%6.2f Y=%6.2f (deg/s)\n",
                  (unsigned long)now, angleRoll, anglePitch, gx, gy, gz);
  }
}

#endif // IMU_ONLY_TEST
