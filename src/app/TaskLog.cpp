/*
  TaskLog.{h,cpp}
  ---------------
  Logging task:
   - Periodically prints RC and attitude state to Serial (10 Hz by default)
   - Low priority to avoid interfering with control loops
*/


#include "TaskLog.h"
#include "Config.h"

static TaskHandle_t sTaskHandle = nullptr;

static void taskLog(void* arg) {
  for (;;) {
    RCState rc;
    AttState att;
    if (xSemaphoreTake(gMutexRC, pdMS_TO_TICKS(5)) == pdTRUE) { rc = gRC; xSemaphoreGive(gMutexRC); }
    if (xSemaphoreTake(gMutexAngles, pdMS_TO_TICKS(5)) == pdTRUE) { att = gAtt; xSemaphoreGive(gMutexAngles); }

    Serial.printf("RC(thr=%u, valid=%d) Att(R=%.2f P=%.2f) Rates(R=%.2f P=%.2f Y=%.2f)\n",
      rc.ch[2], rc.valid, att.angleRoll, att.anglePitch, att.rateRoll, att.ratePitch, att.rateYaw);

    vTaskDelay(pdMS_TO_TICKS(LOG_PERIOD_MS));
  }
}

void startTaskLog() {
  xTaskCreatePinnedToCore(taskLog, "LOG", 3072, nullptr, PRIO_LOG, &sTaskHandle, PRO_CPU_NUM);
}
