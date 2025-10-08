#ifdef EKF_VERT_TEST
/*
  ekf_vert_test.cpp
  -----------------
  Standalone test for vertical EKF using BOTH sensors:
    - MPU6050 (IMU): world-Z linear acceleration, gravity removed (aZlin)
    - BMP280  (Baro): altitude (zeroed at startup)
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <app/Config.h>   // provides SEA_LEVEL_HPA, and likely IMU_PERIOD_MS/BARO_PERIOD_MS

// ---------- Periods / rates ----------
// Use values from Config.h if defined; otherwise provide safe defaults here.
#ifndef IMU_PERIOD_MS
  #define IMU_PERIOD_MS 5     // ~200 Hz
#endif
#ifndef BARO_PERIOD_MS
  #define BARO_PERIOD_MS 40   // ~25 Hz
#endif

static constexpr uint32_t PRINT_MS      = 100;   // 10 Hz prints
static constexpr float    ALT_LPF_ALPHA = 0.15f; // 0..1
static constexpr float    AZ_LPF_ALPHA  = 0.10f; // 0..1

// ---------- EKF tuning ----------
static constexpr float Q_ACC  = 2.0f;        // (m/s^2)^2
static constexpr float Q_BIAS = 0.02f;       // (m/s^2)^2/s
static constexpr float R_BARO = 0.3f * 0.3f; // m^2

// If your IMU is mounted as in your flight code
static constexpr bool IMU_INVERT_YZ = true;

// ---------- Sensors ----------
static Adafruit_BMP280  bmp;
static Adafruit_MPU6050 mpu;

// ---------- Calibration ----------
static float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f; // rad/s
static float accelBiasX = 0.0f, accelBiasY = 0.0f, accelBiasZ = 0.0f;  // m/s²
static float altZero   = 0.0f;  // m
static bool  altZeroSet = false;

// ---------- Attitude & aZlin ----------
static float angleRoll = 0.0f, anglePitch = 0.0f; // deg
static float aZlinLP   = 0.0f;                    // m/s^2 (LPF)

// ---------- Baro LPF ----------
static float altLPF = NAN; // m (zeroed + LPF)

// ---------- EKF state ----------
static bool  ekfInit = false;
static float z_est = 0.0f;    // m
static float vz_est = 0.0f;   // m/s
static float ba_est = 0.0f;   // m/s^2
static float P[3][3] = {
  {10.0f, 0.0f, 0.0f},
  {0.0f, 10.0f, 0.0f},
  {0.0f,  0.0f, 1.0f}
};

// ---------- Helpers ----------
static inline float rad2deg(float r){ return r * 57.2957795f; }
static inline float deg2rad(float d){ return d * 0.01745329252f; }
static inline float lpfStep(float prev, float x, float a){ return isnan(prev) ? x : (prev + a * (x - prev)); }

static void calibrateGyroBias() {
  gyroBiasX = gyroBiasY = gyroBiasZ = 0.0f;
  const int N = 2000;
  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    gyroBiasX += g.gyro.x;
    gyroBiasY += g.gyro.y;
    gyroBiasZ += g.gyro.z;
    delay(1);
  }
  gyroBiasX /= (float)N;
  gyroBiasY /= (float)N;
  gyroBiasZ /= (float)N;
  Serial.printf("[IMU] Gyro bias: (%.6f, %.6f, %.6f) rad/s (N=%d)\n",
                (double)gyroBiasX, (double)gyroBiasY, (double)gyroBiasZ, N);
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
    if (IMU_INVERT_YZ) { ay = -ay; az = -az; }  // Apply inversion here too
    accelBiasX += ax;
    accelBiasY += ay;
    sumZ += az;
    delay(1);
  }
  accelBiasX /= (float)N;
  accelBiasY /= (float)N;
  accelBiasZ = (sumZ / (float)N) - g;  // Z bias = avg_az - g (assume level)
  Serial.printf("[IMU] Accel bias: (%.3f, %.3f, %.3f) m/s² (N=%d)\n",
                (double)accelBiasX, (double)accelBiasY, (double)accelBiasZ, N);
}

static void calibrateAltZero() {
  const int N = 200;
  double sumAlt = 0.0;
  for (int i = 0; i < N; ++i) {
    sumAlt += bmp.readAltitude(SEA_LEVEL_HPA);
    delay(5);
  }
  altZero   = (float)(sumAlt / N);
  altZeroSet = true;
  altLPF    = NAN;
  Serial.printf("[BARO] Altitude zero: %.2f m (N=%d)\n", (double)altZero, N);
}

static void resetEKF() {
  ekfInit = false;
  z_est = vz_est = ba_est = 0.0f;
  P[0][0] = 10.0f; P[0][1] = 0.0f; P[0][2] = 0.0f;
  P[1][0] = 0.0f;  P[1][1] = 10.0f; P[1][2] = 0.0f;
  P[2][0] = 0.0f;  P[2][1] = 0.0f;  P[2][2] = 1.0f;
  Serial.println("[EKF] Reset.");
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== EKF Vertical Test (IMU + Baro) ===");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  bool baroOK = bmp.begin(0x76) || bmp.begin(0x77);
  if (!baroOK) { Serial.println("[BARO] ERROR: BMP280 not found."); while (true) delay(1000); }
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_1
  );

  if (!mpu.begin()) { Serial.println("[IMU] ERROR: MPU6050 not found."); while (true) delay(1000); }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  calibrateGyroBias();
  calibrateAltZero();
  calibrateAccelBias();

  Serial.printf("[CFG] IMU %u Hz  BARO %u Hz  PRINT %u Hz\n",
                (unsigned)(1000 / IMU_PERIOD_MS),
                (unsigned)(1000 / BARO_PERIOD_MS),
                (unsigned)(1000 / PRINT_MS));
  Serial.printf("[EKF] Q_ACC=%.2f  Q_BIAS=%.3f  R_BARO=%.3f\n",
                (double)Q_ACC, (double)Q_BIAS, (double)R_BARO);
}

void loop() {
  // Hotkeys
  if (Serial.available()) {
    char c = (char)Serial.read();
    if (c == 'z' || c == 'Z') calibrateAltZero();
    if (c == 'g' || c == 'G') calibrateGyroBias();
    if (c == 'r' || c == 'R') resetEKF();
  }

  static uint32_t lastImuUs   = 0;
  static uint32_t lastBaroUs  = 0;
  static uint32_t lastPrintMs = 0;

  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  // ----- IMU step (predict + attitude) -----
  if (lastImuUs == 0) lastImuUs = nowUs;
  if ((nowUs - lastImuUs) >= (uint32_t)IMU_PERIOD_MS * 1000UL) {
    float dt = (nowUs - lastImuUs) / 1e6f;
    lastImuUs += (uint32_t)IMU_PERIOD_MS * 1000UL;

    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);

    float ax = a.acceleration.x;
    float ay = a.acceleration.y;
    float az = a.acceleration.z;

    float gx = g.gyro.x - gyroBiasX;  // rad/s
    float gy = g.gyro.y - gyroBiasY;
    float gz = g.gyro.z - gyroBiasZ;

    if (IMU_INVERT_YZ) { ay = -ay; az = -az; gy = -gy; gz = -gz; }

    ax -= accelBiasX;
    ay -= accelBiasY;
    az -= accelBiasZ;

    // Complementary filter for roll/pitch
    float pitchAcc = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.2957795f;
    float rollAcc  = atan2f( ay, az ) * 57.2957795f;
    anglePitch = 0.98f * (anglePitch + rad2deg(gy) * dt) + 0.02f * pitchAcc;
    angleRoll  = 0.98f * (angleRoll  + rad2deg(gx) * dt) + 0.02f * rollAcc;

    // World-Z linear acceleration (gravity removed)
    const float sr = sinf(deg2rad(angleRoll));
    const float cr = cosf(deg2rad(angleRoll));
    const float sp = sinf(deg2rad(anglePitch));
    const float cp = cosf(deg2rad(anglePitch));
    float aZ_world = ax * sp + ay * (-sr * cp) + az * (cr * cp);
    float aZlin    = aZ_world - 9.80665f;
    aZlinLP = lpfStep(aZlinLP, aZlin, AZ_LPF_ALPHA);

    // EKF Predict
    if (ekfInit) {
      z_est  = z_est  + vz_est*dt + 0.5f*(aZlinLP - ba_est)*dt*dt;
      vz_est = vz_est + (aZlinLP - ba_est)*dt;

      float F00 = 1.0f, F01 = dt,   F02 = -0.5f*dt*dt;
      float F10 = 0.0f, F11 = 1.0f, F12 = -dt;
      float F20 = 0.0f, F21 = 0.0f, F22 = 1.0f;

      float dt2 = dt*dt, dt3 = dt2*dt, dt4 = dt2*dt2;
      float Q00 = Q_ACC * (dt4/4.0f);
      float Q01 = Q_ACC * (dt3/2.0f);
      float Q10 = Q01;
      float Q11 = Q_ACC * (dt2);
      float Q22 = Q_BIAS * dt;

      float FP00 = F00*P[0][0] + F01*P[1][0] + F02*P[2][0];
      float FP01 = F00*P[0][1] + F01*P[1][1] + F02*P[2][1];
      float FP02 = F00*P[0][2] + F01*P[1][2] + F02*P[2][2];
      float FP10 = F10*P[0][0] + F11*P[1][0] + F12*P[2][0];
      float FP11 = F10*P[0][1] + F11*P[1][1] + F12*P[2][1];
      float FP12 = F10*P[0][2] + F11*P[1][2] + F12*P[2][2];
      float FP20 = F20*P[0][0] + F21*P[1][0] + F22*P[2][0];
      float FP21 = F20*P[0][1] + F21*P[1][1] + F22*P[2][1];
      float FP22 = F20*P[0][2] + F21*P[1][2] + F22*P[2][2];

      float P00 = FP00*F00 + FP01*F01 + FP02*F02 + Q00;
      float P01 = FP00*F10 + FP01*F11 + FP02*F12 + Q01;
      float P02 = FP00*F20 + FP01*F21 + FP02*F22 + 0.0f;
      float P10 = FP10*F00 + FP11*F01 + FP12*F02 + Q10;
      float P11 = FP10*F10 + FP11*F11 + FP12*F12 + Q11;
      float P12 = FP10*F20 + FP11*F21 + FP12*F22 + 0.0f;
      float P20 = FP20*F00 + FP21*F01 + FP22*F02 + 0.0f;
      float P21 = FP20*F10 + FP21*F11 + FP22*F12 + 0.0f;
      float P22 = FP20*F20 + FP21*F21 + FP22*F22 + Q22;

      P[0][0] = P00; P[0][1] = P01; P[0][2] = P02;
      P[1][0] = P10; P[1][1] = P11; P[1][2] = P12;
      P[2][0] = P20; P[2][1] = P21; P[2][2] = P22;
    }
  }

  // ----- Baro step (update) -----
  if ((nowUs - lastBaroUs) >= (uint32_t)BARO_PERIOD_MS * 1000UL) {
    lastBaroUs = nowUs;

    float altRaw = bmp.readAltitude(SEA_LEVEL_HPA);
    float alt0   = altZeroSet ? (altRaw - altZero) : altRaw;
    altLPF = lpfStep(altLPF, alt0, ALT_LPF_ALPHA);

    // EKF init
    if (!ekfInit && !isnan(altLPF)) {
      z_est = altLPF; vz_est = 0.0f; ba_est = 0.0f;
      ekfInit = true;
      Serial.printf("[EKF] Init: z=%.2f m, vz=0, ba=0\n", (double)z_est);
    }

    // EKF update (H = [1 0 0])
    if (ekfInit && !isnan(altLPF)) {
      float S  = P[0][0] + R_BARO;
      float K0 = P[0][0] / S;
      float K1 = P[1][0] / S;
      float K2 = P[2][0] / S;

      float y = altLPF - z_est;

      z_est  += K0 * y;
      vz_est += K1 * y;
      ba_est += K2 * y;

      float P00n = (1.0f - K0) * P[0][0];
      float P01n = (1.0f - K0) * P[0][1];
      float P02n = (1.0f - K0) * P[0][2];
      float P10n = P[1][0] - K1 * P[0][0];
      float P11n = P[1][1] - K1 * P[0][1];
      float P12n = P[1][2] - K1 * P[0][2];
      float P20n = P[2][0] - K2 * P[0][0];
      float P21n = P[2][1] - K2 * P[0][1];
      float P22n = P[2][2] - K2 * P[0][2];

      P[0][0] = P00n; P[0][1] = P01n; P[0][2] = P02n;
      P[1][0] = P10n; P[1][1] = P11n; P[1][2] = P12n;
      P[2][0] = P20n; P[2][1] = P21n; P[2][2] = P22n;
    }
  }

  // ----- Prints -----
  if ((nowMs - lastPrintMs) >= PRINT_MS) {
    lastPrintMs = nowMs;

    float T = bmp.readTemperature();
    float PhPa = bmp.readPressure() * 0.01f;
    float altRaw = bmp.readAltitude(SEA_LEVEL_HPA);
    float alt0   = altZeroSet ? (altRaw - altZero) : altRaw;

    Serial.printf("t=%lu ms | T=%6.2f °C  P=%8.2f hPa  "
                  "AltRaw=%8.2f m  Alt0LPF=%8.2f m  "
                  "aZlin=%7.3f m/s^2  "
                  "EKF z=%8.2f m  vz=%7.3f m/s  ba=%6.3f m/s^2\n",
                  (unsigned long)nowMs,
                  (double)T, (double)PhPa,
                  (double)altRaw, (double)altLPF,
                  (double)aZlinLP,
                  (double)z_est, (double)vz_est, (double)ba_est);
  }
}

#endif // EKF_VERT_TEST
