/*
  main.cpp
  --------

  === Task Flow ===
  1) TaskRC      : reads RX → gRC
  2) TaskIMU     : attitude + aZlin → gAtt, gVertAcc
  3) TaskBaro    : altitude (zeroed + LPF) → gBaro
  4) TaskControl : cascaded attitude control + vertical EKF (uses gVertAcc + gBaro)
  5) TaskLog     : periodic prints (debug/monitoring)
*/

#if !defined(IMU_ONLY_TEST) && !defined(BARO_ONLY_TEST) && !defined(EKF_VERT_TEST)

#include <Arduino.h>
#include <Wire.h>
#include "app/Shared.h"
#include "app/Config.h"
#include "app/TaskRC.h"
#include "app/TaskIMU.h"
// startTaskBaro is declared in Shared.h; no extra header required.
#include "app/TaskControl.h"
#include "app/TaskLog.h"

void setup() {
  Serial.begin(115200);
  delay(200);

  // --- Create queues & mutexes ---
  gMutexAngles  = xSemaphoreCreateMutex();
  gMutexRC      = xSemaphoreCreateMutex();
  gMutexMotors  = xSemaphoreCreateMutex();
  gMutexBaro    = xSemaphoreCreateMutex();
  gMutexI2C     = xSemaphoreCreateMutex();
  gMutexVertAcc = xSemaphoreCreateMutex();
  gQPPM         = xQueueCreate(64, sizeof(PpmPulse));

  // --- Init I2C before any task touches the bus ---
  if (xSemaphoreTake(gMutexI2C, portMAX_DELAY) == pdTRUE) {
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    xSemaphoreGive(gMutexI2C);
  }

  // --- Start tasks ---
  // Order: sensors first (IMU, Baro) so Control has data on first iterations.
  startTaskRC();
  startTaskIMU();
  startTaskBaro();      // <-- needed for altitude (EKF update)
  startTaskControl();
  startTaskLog();

  Serial.println("FreeRTOS tasks started.");
}

void loop() {
  // All work happens in tasks.
}

#endif // !IMU_ONLY_TEST && !BARO_ONLY_TEST && !EKF_VERT_TEST