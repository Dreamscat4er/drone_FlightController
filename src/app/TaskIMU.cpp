#include "TaskIMU.h"
#include "Utils.h"
#include "Config.h"
#include "Shared.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

static TaskHandle_t sTaskHandle = nullptr;
static Adafruit_MPU6050 mpu;

// ---------- Calibration ----------
static float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f; // rad/s
static float accelBiasX = 0.0f, accelBiasY = 0.0f, accelBiasZ = 0.0f;  // m/s²

/** Use RTOS-friendly wait in calibration. */
static void calibrateGyro() {
  gyroBiasX = gyroBiasY = gyroBiasZ = 0;
  for (int i = 0; i < 2000; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    gyroBiasX += g.gyro.x;
    gyroBiasY += g.gyro.y;
    gyroBiasZ += g.gyro.z;
    vTaskDelay(1);
  }
  gyroBiasX /= 2000.0f;
  gyroBiasY /= 2000.0f;
  gyroBiasZ /= 2000.0f;
  Serial.printf("[IMU] Gyro bias: (%.6f, %.6f, %.6f) rad/s (N=2000)\n",
                (double)gyroBiasX, (double)gyroBiasY, (double)gyroBiasZ);
}

static void calibrateAccelBias() {
  accelBiasX = accelBiasY = 0.0f;
  double sumZ = 0.0;
  const int N = 2000;  // Match gyro samples
  const float g = 9.80665f;
  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;
    // IMU is mounted upside down (your existing convention)
    ay = -ay;
    az = -az;
    accelBiasX += ax;
    accelBiasY += ay;
    sumZ += az;
    vTaskDelay(1);
  }
  accelBiasX /= (float)N;
  accelBiasY /= (float)N;
  accelBiasZ = (sumZ / (float)N) - g;  // Z bias = avg_az - g (assume level)
  Serial.printf("[IMU] Accel bias: (%.3f, %.3f, %.3f) m/s² (N=%d)\n",
                (double)accelBiasX, (double)accelBiasY, (double)accelBiasZ, N);
}

static void taskIMU(void* arg) {

  // ---- MPU6050 init ----
  if (xSemaphoreTake(gMutexI2C, portMAX_DELAY) == pdTRUE) {
    if (!mpu.begin()) {
      xSemaphoreGive(gMutexI2C);
      Serial.println("MPU6050 not found! IMU task halted.");
      vTaskDelete(nullptr);
    }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    xSemaphoreGive(gMutexI2C);
  }

  calibrateGyro();
  calibrateAccelBias();

  float angleRoll = 0, anglePitch = 0;

  // --- optional small LPF state for aZlin ---
  static float aZlinLP = 0.0f;
  const float AZ_ALPHA = 0.10f;      // ~10 Hz at 200 Hz loop (tune as needed)

  const TickType_t period = pdMS_TO_TICKS(IMU_PERIOD_MS);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&last, period);

    // ===== IMU read & fuse (e.g., 200 Hz) =====
    sensors_event_t a, g, t;
    if (xSemaphoreTake(gMutexI2C, pdMS_TO_TICKS(2)) == pdTRUE) {
      mpu.getEvent(&a, &g, &t);
      xSemaphoreGive(gMutexI2C);
    } else {
      continue;
    }

    // Adafruit units: acceleration in m/s^2, gyro in rad/s
    float accelX = a.acceleration.x;
    float accelY = a.acceleration.y;
    float accelZ = a.acceleration.z;

    float gx = g.gyro.x - gyroBiasX;  // rad/s (sub before invert)
    float gy = g.gyro.y - gyroBiasY;
    float gz = g.gyro.z - gyroBiasZ;

    // IMU is mounted upside down (your existing convention)
    accelY = -accelY;
    accelZ = -accelZ;
    gy  = -gy;
    gz  = -gz;

    // Subtract accel biases AFTER inversion (to match cal frame)
    accelX -= accelBiasX;
    accelY -= accelBiasY;
    accelZ -= accelBiasZ;

    float gyroX = applyDeadband(gx * RAD_TO_DEG, 0.5f);
    float gyroY = applyDeadband(gy * RAD_TO_DEG, 0.5f);
    float gyroZ = applyDeadband(gz * RAD_TO_DEG, 0.5f);

    // --- complementary filter for roll/pitch (as before) ---
    float pitchAcc = applyDeadband(atan2f(-accelX, sqrtf(accelY*accelY + accelZ*accelZ)) * RAD_TO_DEG, 5);
    float rollAcc  = applyDeadband(atan2f(accelY, accelZ) * RAD_TO_DEG, 5);

    const float dt = IMU_PERIOD_MS / 1000.0f;
    anglePitch = 0.98f * (anglePitch + gyroY * dt) + 0.02f * pitchAcc;
    angleRoll  = 0.98f * (angleRoll  + gyroX * dt) + 0.02f * rollAcc;

    // --- publish attitude ---
    if (xSemaphoreTake(gMutexAngles, pdMS_TO_TICKS(1)) == pdTRUE) {
      gAtt.angleRoll  = angleRoll;
      gAtt.anglePitch = anglePitch;
      gAtt.rateRoll   = gyroX;
      gAtt.ratePitch  = gyroY;
      gAtt.rateYaw    = gyroZ;
      gAtt.stampUs    = micros();
      xSemaphoreGive(gMutexAngles);
    }

    // --- NEW: compute world-Z linear acceleration (gravity removed) ---
    // Only roll/pitch are needed for world-Z.
    const float sr = sinf(angleRoll  * DEG_TO_RAD);
    const float cr = cosf(angleRoll  * DEG_TO_RAD);
    const float sp = sinf(anglePitch * DEG_TO_RAD);
    const float cp = cosf(anglePitch * DEG_TO_RAD);

    // World Z acceleration from body accel (using your sign conventions):
    // a_z_world = ax*sp + ay*(-sr*cp) + az*(cr*cp)
    float aZ_world = accelX * sp + accelY * (-sr * cp) + accelZ * (cr * cp);

    // Remove gravity (+Z up). Adafruit accel is m/s^2.
    const float g0 = 9.80665f;
    float aZlin = aZ_world - g0;

    // Light LPF to fight motor/prop vibration
    aZlinLP += AZ_ALPHA * (aZlin - aZlinLP);

    // --- publish vertical acceleration for EKF ---
    if (xSemaphoreTake(gMutexVertAcc, pdMS_TO_TICKS(1)) == pdTRUE) {
      gVertAcc.aZlin   = aZlinLP;        // m/s^2, +up, gravity removed
      gVertAcc.stampUs = micros();
      gVertAcc.valid   = true;
      xSemaphoreGive(gMutexVertAcc);
    }
  }
}

void startTaskIMU() {
  xTaskCreatePinnedToCore(taskIMU, "IMU", 4096, nullptr, PRIO_IMU, &sTaskHandle, PRO_CPU_NUM);
}