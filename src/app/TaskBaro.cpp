#include "TaskBaro.h"
#include "Shared.h"
#include "Config.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>

/*
 * Baro task
 * - Initializes BMP280 on the shared I2C bus
 * - Calibrates altitude only: averages N altitude samples to define zero at startup
 * - Reads T/P/altitude at BARO_PERIOD_MS
 * - Optional gentle LPF on zeroed altitude
 * - Publishes BaroState (EKF uses altitude directly)
 */

static TaskHandle_t sBaroHandle = nullptr;
static Adafruit_BMP280 bmp; // I2C

// Altitude LPF state
static float sAltLP   = NAN;
// Altitude zero (m) set at startup calibration
static float sAltZero = 0.0f;
static bool  sCalDone = false;


// --- Altitude-only calibration: average N altitude samples ---
static void calibrateAltZero() {
  double sumAlt = 0.0;
  for (int i = 0; i < 200; ++i) {
    float alt = bmp.readAltitude(SEA_LEVEL_HPA);
    sumAlt += alt;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  sAltZero = (float)(sumAlt / 200);
  sCalDone = true;
  Serial.printf("[BARO] Altitude zero: %.2f m (N=200)\n", (double)sAltZero);
}

static void taskBaro(void*) {
  // Init BMP280 on shared I2C
  if (xSemaphoreTake(gMutexI2C, portMAX_DELAY) == pdTRUE) {
    bool ok = bmp.begin(0x76) || bmp.begin(0x77);
    if (ok) {
      bmp.setSampling(
        Adafruit_BMP280::MODE_NORMAL,
        Adafruit_BMP280::SAMPLING_X2,    // temp OS
        Adafruit_BMP280::SAMPLING_X16,   // pressure OS
        Adafruit_BMP280::FILTER_X16,     // strong IIR
        Adafruit_BMP280::STANDBY_MS_1    // responsive
      );
    } else {
      xSemaphoreGive(gMutexI2C);
      Serial.println("BMP280 not found! Baro task halted.");
      vTaskDelete(nullptr);
    }
    xSemaphoreGive(gMutexI2C);
  }

  // ---- Altitude-only calibration (define ground reference) ----
  if (xSemaphoreTake(gMutexI2C, portMAX_DELAY) == pdTRUE) {
    calibrateAltZero();
    xSemaphoreGive(gMutexI2C);
  }

  const TickType_t period = pdMS_TO_TICKS(BARO_PERIOD_MS);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&last, period);

    float T = NAN, P = NAN, altRaw = NAN;
    if (xSemaphoreTake(gMutexI2C, pdMS_TO_TICKS(5)) == pdTRUE) {
      T      = bmp.readTemperature();              // °C
      P      = bmp.readPressure();                 // Pa
      altRaw = bmp.readAltitude(SEA_LEVEL_HPA);    // meters (+up)
      xSemaphoreGive(gMutexI2C);
    } else {
      continue; // try next tick
    }

    // Zeroed altitude (relative to startup ground)
    float alt0 = (!isnan(altRaw) && sCalDone) ? (altRaw - sAltZero) : altRaw;
  
    // gentle LPF on zeroed altitude. Include after testing  
  /*   if (!isnan(alt0)) {
      const float Z_ALPHA = 0.15f; // ~2–4 Hz
      if (isnan(sAltLP)) sAltLP = alt0;
      else               sAltLP += Z_ALPHA * (alt0 - sAltLP);
    } */

    if (xSemaphoreTake(gMutexBaro, pdMS_TO_TICKS(2)) == pdTRUE) {
      gBaro.temperature = T;
      gBaro.pressure    = P;
      gBaro.altitude    = isnan(sAltLP) ? alt0 : sAltLP;  // publish zeroed (or LPF’d) altitude
      gBaro.stampUs     = micros();
      gBaro.valid       = !isnan(T) && !isnan(P) && !isnan(altRaw);
      xSemaphoreGive(gMutexBaro);
    }
  }
}

void startTaskBaro() {
  xTaskCreatePinnedToCore(taskBaro, "BARO", 4096, nullptr, PRIO_BARO, &sBaroHandle, PRO_CPU_NUM);
}