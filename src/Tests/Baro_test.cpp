/*
  baro_test.cpp
  -------------------
  BARO-only test entry point.

  Compiled only when BARO_ONLY_TEST is defined (env:baro_test).

  What it does:
    • Initializes BMP280 on I2C
    • Calibrates altitude only (averages N samples) to define zero (ground reference)
    • Reads temperature (°C), pressure (hPa), altitude (m)
    • Prints raw altitude and zeroed/LPF altitude at a steady rate

  Tip: press 'z' in Serial Monitor to re-zero altitude at any time.
*/

#ifdef BARO_ONLY_TEST

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <app/Config.h>  

// ---------- User-tunable params ----------
static constexpr uint32_t PRINT_MS        = 100;    // serial print rate
static constexpr float    ALT_LPF_ALPHA   = 0.15f;  // 0..1, 0 disables LPF
static constexpr int      CAL_SAMPLES     = 2000;    // samples for zeroing altitude
static constexpr uint32_t CAL_DELAY_MS    = 5;      // delay between cal samples

// ---------- Globals ----------
static Adafruit_BMP280 bmp;        // I2C barometer
static uint8_t usedAddr = 0;       // 0x76 or 0x77 (detected)
static float altZero = 0.0f;       // altitude zero offset (m), computed at startup
static bool  calDone = false;      // set true after calibration
static float altLPF  = NAN;        // LPF state for zeroed altitude

// ---------- Helpers ----------
static inline float lpfStep(float prev, float input, float alpha) {
  return isnan(prev) ? input : (prev + alpha * (input - prev));
}

// Altitude-only calibration: average N altitude samples and store as zero
static void calibrateAltZero() {
  double sumAlt = 0.0;
  for (int i = 0; i < CAL_SAMPLES; ++i) {
    float alt = bmp.readAltitude(SEA_LEVEL_HPA);  // meters (+up)
    sumAlt += alt;
    delay(CAL_DELAY_MS);
  }
  altZero = (float)(sumAlt / CAL_SAMPLES);
  calDone = true;

  Serial.printf("[BARO] Altitude zero set to %.2f m (N=%d)\n", (double)altZero, CAL_SAMPLES);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== BARO Only Test (BARO_ONLY_TEST) ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // Try both common I2C addresses
  bool ok = false;
  if (bmp.begin(0x76)) { usedAddr = 0x76; ok = true; }
  else if (bmp.begin(0x77)) { usedAddr = 0x77; ok = true; }

  if (!ok) {
    Serial.println("[BARO] ERROR: BMP280 not found at 0x76 or 0x77!");
    while (true) delay(1000);
  }

  // Match flight settings
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,     // temperature OS
    Adafruit_BMP280::SAMPLING_X16,    // pressure OS
    Adafruit_BMP280::FILTER_X16,      // strong IIR
    Adafruit_BMP280::STANDBY_MS_1     // responsive
  );

  Serial.printf("[BARO] Sensor OK @ 0x%02X\n", usedAddr);
  Serial.printf("[BARO] BARO_PERIOD_MS=%u, PRINT_MS=%u, ALT_LPF_ALPHA=%.2f, SEA_LEVEL_HPA=%.2f\n",
                (unsigned)BARO_PERIOD_MS, (unsigned)PRINT_MS,
                (double)ALT_LPF_ALPHA, (double)SEA_LEVEL_HPA);

  // ---- Altitude-only calibration (define ground reference) ----
  calibrateAltZero();
}

void loop() {
  // Hotkey: press 'z' in serial to re-zero altitude
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'z' || c == 'Z') {
      calibrateAltZero();
      altLPF = NAN; // reset LPF to avoid a step transient
    }
  }

  static uint32_t lastSampleUs = 0;
  static uint32_t lastPrintMs  = 0;
  const uint32_t  periodUs     = BARO_PERIOD_MS * 1000UL;

  if (lastSampleUs == 0) lastSampleUs = micros();
  while ((micros() - lastSampleUs) < periodUs) { /* spin-wait to keep cadence */ }
  lastSampleUs += periodUs;

  // ---- Read sensor ----
  const float T     = bmp.readTemperature();            // °C
  const float PhPa  = bmp.readPressure() * 0.01f;       // hPa
  const float altR  = bmp.readAltitude(SEA_LEVEL_HPA);  // raw altitude (m)
  const float alt0  = calDone ? (altR - altZero) : altR;// zeroed altitude

  // ---- Optional LPF on zeroed altitude ----
  const bool lpfOn = (ALT_LPF_ALPHA > 0.0f && ALT_LPF_ALPHA <= 1.0f);
  if (lpfOn) altLPF = lpfStep(altLPF, alt0, ALT_LPF_ALPHA);
  else       altLPF = alt0;

  // ---- Print ----
  const uint32_t nowMs = millis();
  if (nowMs - lastPrintMs >= PRINT_MS) {
    lastPrintMs = nowMs;
    Serial.printf("t=%lu ms | T=%6.2f °C  P=%8.2f hPa  AltRaw=%8.2f m  Alt0=%8.2f m  AltLPF=%8.2f m\n",
                  (unsigned long)nowMs, (double)T, (double)PhPa,
                  (double)altR, (double)alt0, (double)altLPF);
  }
}

#endif // BARO_ONLY_TEST
