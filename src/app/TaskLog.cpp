/*
  TaskLog.{h,cpp}
  ---------------
  Logging task:
   - Periodically prints RC, attitude, and motor outputs to Serial (10 Hz by default)
   - Low priority to avoid interfering with control loops
*/

#include "TaskLog.h"
#include "Config.h"
#include "Shared.h"   // <-- needed for gRC, gAtt, gMotors and their mutexes

static TaskHandle_t sTaskHandle = nullptr;

static void taskLog(void* arg) {
  for (;;) {
    // Take snapshots under mutexes so we print a consistent view
    RCState rc;
    AttState att;
    MotorFeed motors;

    if (xSemaphoreTake(gMutexRC, pdMS_TO_TICKS(5)) == pdTRUE) {
      rc = gRC;
      xSemaphoreGive(gMutexRC);
    }

    if (xSemaphoreTake(gMutexAngles, pdMS_TO_TICKS(5)) == pdTRUE) {
      att = gAtt;
      xSemaphoreGive(gMutexAngles);
    }

    if (xSemaphoreTake(gMutexMotors, pdMS_TO_TICKS(5)) == pdTRUE) {
      motors = gMotors;
      xSemaphoreGive(gMutexMotors);
    }

    // Print a compact line: RC, Attitude, Rates, Motors (µs)
    Serial.printf(
      "RC(thr=%u, valid=%d) Att(R=%.2f P=%.2f) Rates(R=%.2f P=%.2f Y=%.2f) "
      "Motors(us): M1=%u M2=%u M3=%u M4=%u\n",
      rc.ch[2], rc.valid,
      att.angleRoll, att.anglePitch, att.rateRoll, att.ratePitch, att.rateYaw,
      motors.m1, motors.m2, motors.m3, motors.m4
    );

    vTaskDelay(pdMS_TO_TICKS(LOG_PERIOD_MS));
  }
}

void startTaskLog() {
  xTaskCreatePinnedToCore(taskLog, "LOG", 3072, nullptr, PRIO_LOG, &sTaskHandle, PRO_CPU_NUM);
}
