/*
  TaskRC.{h,cpp}
  --------------
  Handles RC receiver input via PPM:
   - ISR captures pulse widths
   - TaskRC assembles channel data into RCState
   - Provides failsafe if no valid frame in >100 ms
*/


#include "TaskRC.h"
#include "Config.h"

static TaskHandle_t sTaskHandle = nullptr;
static volatile uint32_t sIsrLastTime = 0;

static void taskRC(void* arg) {
  PpmPulse p;
  uint16_t frame[CHANNELS] = {0};
  uint8_t idx = 0;
  uint32_t lastPacketUs = micros();

  for (;;) {
    if (xQueueReceive(gQPPM, &p, pdMS_TO_TICKS(20)) == pdTRUE) {
      if (p.eof) {
        if (idx == CHANNELS) {
          if (xSemaphoreTake(gMutexRC, pdMS_TO_TICKS(2)) == pdTRUE) {
            memcpy(gRC.ch, frame, sizeof(frame));
            gRC.lastUpdateUs = micros();
            gRC.valid = true;
            xSemaphoreGive(gMutexRC);
          }
        }
        idx = 0;
      } else if (idx < CHANNELS) {
        frame[idx++] = p.widthUs;
      }
      lastPacketUs = micros();
    }
    // Failsafe
    if ((micros() - lastPacketUs) > 100000) {
      if (xSemaphoreTake(gMutexRC, pdMS_TO_TICKS(2)) == pdTRUE) {
        gRC.valid = false;
        xSemaphoreGive(gMutexRC);
      }
      lastPacketUs = micros();
    }
  }
}

void IRAM_ATTR handlePPMInterrupt() {
  uint32_t now = micros();
  uint32_t pulse = now - sIsrLastTime;
  sIsrLastTime = now;

  PpmPulse p{ (uint16_t)pulse, pulse > PPM_EOF };
  BaseType_t higher = pdFALSE;
  if (gQPPM) xQueueSendFromISR(gQPPM, &p, &higher);
  if (higher) portYIELD_FROM_ISR();
}

void startTaskRC() {
  // Queue & pin/interrupt
  if (!gQPPM) gQPPM = xQueueCreate(64, sizeof(PpmPulse));
  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), handlePPMInterrupt, RISING);

  xTaskCreatePinnedToCore(taskRC, "RC", 3072, nullptr, PRIO_RC, &sTaskHandle, APP_CPU_NUM);
}
