/*
  TaskIMU.{h,cpp}
  ---------------
  IMU task:
   - Initializes MPU6050 (I2C)
   - Calibrates gyro biases
   - Reads sensor events at 200 Hz
   - Runs complementary filter to estimate roll/pitch
   - Publishes AttState with angles and rates
*/


#include "TaskIMU.h"
#include "Utils.h"
#include "Config.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

static TaskHandle_t sTaskHandle = nullptr;
static Adafruit_MPU6050 mpu;

static void calibrateGyro() {
  gyroBiasX = gyroBiasY = gyroBiasZ = 0;
  for (int i = 0; i < 2000; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    gyroBiasX += g.gyro.x;
    gyroBiasY += g.gyro.y;
    gyroBiasZ += g.gyro.z;
    delay(1);
  }
  gyroBiasX /= 2000.0f;
  gyroBiasY /= 2000.0f;
  gyroBiasZ /= 2000.0f;
}

static void taskIMU(void* arg) {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found! IMU task halted.");
    vTaskDelete(nullptr);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  calibrateGyro();

  float angleRoll = 0, anglePitch = 0;
  const TickType_t period = pdMS_TO_TICKS(IMU_PERIOD_MS);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&last, period);

    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    float accelX = a.acceleration.x;
    float accelY = a.acceleration.y;
    float accelZ = a.acceleration.z;

    float gyroX = applyDeadband((g.gyro.x - gyroBiasX) * RAD_TO_DEG, 0.5f);
    float gyroY = applyDeadband((g.gyro.y - gyroBiasY) * RAD_TO_DEG, 0.5f);
    float gyroZ = applyDeadband((g.gyro.z - gyroBiasZ) * RAD_TO_DEG, 0.5f);

    float pitchAcc = applyDeadband(atan2f(-accelX, sqrtf(accelY*accelY + accelZ*accelZ)) * RAD_TO_DEG, 5);
    float rollAcc  = applyDeadband(atan2f(accelY, accelZ) * RAD_TO_DEG, 5);

    const float dt = IMU_PERIOD_MS / 1000.0f;
    anglePitch = 0.98f * (anglePitch + gyroY * dt) + 0.02f * pitchAcc;
    angleRoll  = 0.98f * (angleRoll  + gyroX * dt) + 0.02f * rollAcc;

    if (xSemaphoreTake(gMutexAngles, pdMS_TO_TICKS(1)) == pdTRUE) {
      gAtt.angleRoll = angleRoll;
      gAtt.anglePitch = anglePitch;
      gAtt.rateRoll = gyroX;
      gAtt.ratePitch = gyroY;
      gAtt.rateYaw = gyroZ;
      gAtt.stampUs = micros();
      xSemaphoreGive(gMutexAngles);
    }
  }
}

void startTaskIMU() {
  xTaskCreatePinnedToCore(taskIMU, "IMU", 4096, nullptr, PRIO_IMU, &sTaskHandle, PRO_CPU_NUM);
}
