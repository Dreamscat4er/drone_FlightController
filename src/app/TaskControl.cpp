/*
  TaskControl.cpp
  ---------------
  PURPOSE (short):
    This task runs the flight control loop (attitude + yaw) and manages vertical motion.
    Vertical control relies purely on velocity (vz_est from EKF), ignoring altitude drift.
    - Stick up: target positive vz (climb).
    - Stick down: target negative vz (descend).
    - Stick middle (deadband): targetVz=0 for hover (counters small vz_est drift via PID).
    EKF still fuses IMU/baro for reliable vz_est (baro helps long-term, but we don't hold position).

  EKF MATH (nutshell):
    The EKF keeps an internal “belief” about the vertical motion in a State vector:       
                        x_k = [ z_k, vz_k, ba_k ]^T
                        z  : altitude at time step k (m, +up) — tracked but NOT used for control
                        vz : vertical velocity at time step k (m/s, +up)
                        ba : accel bias at time step k(m/s²), slow offset

    Discrete process model. Predicts how the state changes from one control tick to the next using physics
    (constant-acceleration kinematics) and the IMU input aZlin (vertical acceleration, gravity removed). With timestep dt::
      z_{k+1}  = z_k  + vz_k * dt + 0.5 * (aZlin_k - ba_k) * dt^2
      vz_{k+1} = vz_k + (aZlin_k - ba_k) * dt
      ba_{k+1} = ba_k   (assumed bias changes very slowly; its drift is modeled in Q)

    Measurement model:
      z_meas = z + noise        (baro measures altitude directly)
      H = [1 0 0]

    EKF loop each tick:
      Predict:
        x ← f(x, aZlin)         (apply the kinematic equations above)
        P ← F P Fᵀ + Q          (F = ∂f/∂x; Q from accel noise + bias random-walk)
      Update (if baro fresh):
        y  = z_meas − Hx        (innovation)
        S  = H P Hᵀ + R         (innovation covariance)
        K  = P Hᵀ S⁻¹           (Kalman gain)
        x ← x + K y             (state correction)
        P ← (I − K H) P         (covariance correction)
*/

#include "TaskControl.h"
#include "Utils.h"
#include "Config.h"
#include "Shared.h"
#include <math.h>

static TaskHandle_t sTaskHandle = nullptr;

// -------------------- EKF TUNING --------------------
// Q_ACC  : accel noise drive (m/s^2)^2 → larger = smoother, smaller = faster
// Q_BIAS : accel-bias random walk strength (m/s^2)^2/s → larger = bias adapts faster
// R_BARO : baro altitude variance (m^2) → larger = trust baro less (keep low for vz accuracy)
static const float Q_ACC  = 2.0f;
static const float Q_BIAS = 0.02f;
static const float R_BARO = 0.3f * 0.3f;
// ---------------------------------------------------

// Dynamic hover throttle estimation
static const float HOVER_EST_ALPHA = 0.1f;  // Low-pass alpha for slow adaptation
static const float HOVER_VZ_THRESH = 0.05f; // |vz_est| < this for "hover" detection (m/s)
static const uint32_t HOVER_SAMPLE_MS = 50; // Sample every 50 ms to avoid noise

// Helper clamp for ESC pulses
static inline int clampi(int v, int lo, int hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

// Simple LPF helper (if not in Utils.h)
static inline float lpfStep(float prev, float x, float alpha) {
  return prev + alpha * (x - prev);
}

static void taskControl(void* arg) {
  // ---- PWM setup (ESC outputs) ----
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(3, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(4, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM1_PIN, 1);
  ledcAttachPin(PWM2_PIN, 2);
  ledcAttachPin(PWM3_PIN, 3);
  ledcAttachPin(PWM4_PIN, 4);

  const TickType_t period = pdMS_TO_TICKS(CTRL_PERIOD_MS); // e.g., 5 ms → 200 Hz
  TickType_t last = xTaskGetTickCount();

  // Dynamic hover estimation state
  static float estHoverThr = 1500.0f;  // Initial guess (µs)
  static float thrSum = 0.0f;
  static int hoverCount = 0;
  static uint32_t lastHoverMs = 0;

  // -------------------------- EKF STATE --------------------------
  static bool  ekfInit = false; // becomes true after first valid init

  static float z_est = 0.0f;    // [EKF state] altitude estimate (m, +up) — tracked but NOT used for control
  static float vz_est = 0.0f;   // [EKF state] vertical speed estimate (m/s, +up) — PRIMARY feedback
  static float ba_est = 0.0f;   // [EKF state] accel bias estimate (m/s^2)

  // P is the 3×3 covariance matrix of estimation uncertainty:
  // rows/cols correspond to [z, vz, ba] in that order.
  //   P(0,0): variance of z (m^2)
  //   P(1,1): variance of vz ((m/s)^2)
  //   P(2,2): variance of ba ((m/s^2)^2)
  // Off-diagonals encode correlations (start at 0).
  //
  // Initialization choice:
  //   - z, vz start fairly uncertain (10): the filter will quickly correct with baro/IMU.
  //   - ba starts modest (1): bias adapts, but not too aggressively at boot.
  static float P[3][3] = {
    {10.0f, 0.0f, 0.0f},
    {0.0f, 10.0f, 0.0f},
    {0.0f,  0.0f, 1.0f}
  };
  // ---------------------------------------------------------------

  for (;;) {
    vTaskDelayUntil(&last, period);

    // --------- Copy shared inputs ---------
    RCState rc;
    AttState att;
    BaroState baro;
    VertAccState va;

    if (xSemaphoreTake(gMutexRC,      pdMS_TO_TICKS(1)) == pdTRUE) { rc   = gRC;      xSemaphoreGive(gMutexRC);      } else continue;
    if (xSemaphoreTake(gMutexAngles,  pdMS_TO_TICKS(1)) == pdTRUE) { att  = gAtt;     xSemaphoreGive(gMutexAngles);  } else continue;
    if (xSemaphoreTake(gMutexBaro,    pdMS_TO_TICKS(1)) == pdTRUE) { baro = gBaro;    xSemaphoreGive(gMutexBaro);    } else continue;
    if (xSemaphoreTake(gMutexVertAcc, pdMS_TO_TICKS(1)) == pdTRUE) { va   = gVertAcc; xSemaphoreGive(gMutexVertAcc); } else continue;

    // -------------- RC mapping --------------
    float targetRoll    = map(rc.ch[0], 1000, 2000, -MAX_ANGLE,    MAX_ANGLE);
    float targetPitch   = map(rc.ch[1], 1000, 2000, -MAX_ANGLE,    MAX_ANGLE);
    float vzStick       = rc.ch[2];
    float targetYawRate = map(rc.ch[3], 1000, 2000, -MAX_YAW_RATE, MAX_YAW_RATE);
    targetRoll    = applyDeadband(targetRoll, 1.0f);
    targetPitch   = applyDeadband(targetPitch, 1.0f);
    targetYawRate = applyDeadband(targetYawRate, 1.0f);
    float targetVz = map(vzStick, 1000, 2000, -MAX_CLIMB_RATE, MAX_CLIMB_RATE);
    targetVz = applyDeadband(targetVz, VZ_DEADBAND);  // Middle stick → 0 (hover)
    if (!rc.valid) targetVz = 0.0f;
    const float dt = CTRL_PERIOD_MS / 1000.0f;
    // ---------------------------------------

    // --------- Attitude control (unchanged) ----------
    float desiredRateRoll  = computePID(targetRoll,  att.angleRoll,  integralAngleRoll,  prevErrorAngleRoll,
                                        Kp_angle, Ki_angle, Kd_angle, dt);
    float desiredRatePitch = computePID(targetPitch, att.anglePitch, integralAnglePitch, prevErrorAnglePitch,
                                        Kp_angle, Ki_angle, Kd_angle, dt);
    float rollOut  = computePID(desiredRateRoll,  att.rateRoll,  integralRoll,  prevErrorRoll,
                                Kp_rate, Ki_rate, Kd_rate, dt);
    float pitchOut = computePID(desiredRatePitch, att.ratePitch, integralPitch, prevErrorPitch,
                                Kp_rate, Ki_rate, Kd_rate, dt);
    float yawOut   = computePID(targetYawRate,    att.rateYaw,   integralYaw,   prevErrorYaw,
                                Kp_yaw,  Ki_yaw,  Kd_yaw,  dt);
    // -------------------------------------------------

    // ==================== EKF ======================
    // Init once with valid baro + accel-Z
    if (!ekfInit && baro.valid && va.valid) {
      z_est  = baro.altitude; // start from baro altitude (for tracking only)
      vz_est = 0.0f;          // assume stationary vertically at start
      ba_est = 0.0f;          // EKF will learn bias via updates
      ekfInit = true;
      Serial.printf("[EKF] Init: z=%.2f m, vz=0, ba=0\n", (double)z_est);
    }

    if (ekfInit) {
      // --- Predict ---
      const float az = va.valid ? va.aZlin : 0.0f; // world-Z accel, gravity removed (m/s^2)
      z_est  = z_est  + vz_est*dt + 0.5f*(az - ba_est)*dt*dt;
      vz_est = vz_est + (az - ba_est)*dt;
      // ba_est unchanged here; bias drift enters via Q_BIAS

      // F = ∂f/∂x  for x=[z,vz,ba]
      float F00 = 1.0f, F01 = dt,   F02 = -0.5f*dt*dt;
      float F10 = 0.0f, F11 = 1.0f, F12 = -dt;
      float F20 = 0.0f, F21 = 0.0f, F22 = 1.0f;

      // Q (discrete process noise)
      const float q  = Q_ACC;
      const float qb = Q_BIAS;
      float dt2 = dt*dt, dt3 = dt2*dt, dt4 = dt2*dt2;
      float Q00 = q * (dt4/4.0f);
      float Q01 = q * (dt3/2.0f);
      float Q10 = Q01;
      float Q11 = q * (dt2);
      float Q22 = qb * dt;

      // P ← F P Fᵀ + Q
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

      // --- Update (baro altitude) ---
      const bool baroFresh = (uint32_t)(micros() - baro.stampUs) < 150000;
      if (baro.valid && baroFresh) {
        // H = [1 0 0]
        float S  = P[0][0] + R_BARO;    // innovation covariance
        float K0 = P[0][0] / S;         // Kalman gains
        float K1 = P[1][0] / S;
        float K2 = P[2][0] / S;

        float y = baro.altitude - z_est; // innovation

        // x ← x + K y
        z_est  += K0 * y;
        vz_est += K1 * y;
        ba_est += K2 * y;

        // P ← (I − K H) P  with H = [1 0 0]
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
    // ===========================================

    // ---- Pure Vertical Velocity PID: use EKF vz_est as feedback ----
    // targetVz from stick: up=climb, down=descend, middle=0 (hover, counters drift)
    // Deadband in PID handles small vz_est noise (~±0.05 m/s)
    float throttleDeltaUs = computePID(
      targetVz, vz_est,
      integralVz, prevErrorVz,
      Kp_vz, Ki_vz, Kd_vz,
      dt,
      VZ_OUT_MIN, VZ_OUT_MAX,
      VZ_DEADBAND  // Clamps small errors around 0
    );

    // ---- Dynamic Hover Estimation ----
    const uint32_t nowMs = millis();
    if (fabsf(targetVz) < VZ_DEADBAND && fabsf(vz_est) < HOVER_VZ_THRESH) {  // Detected hover
      if (nowMs - lastHoverMs > HOVER_SAMPLE_MS) {  // Sample every 50 ms
        thrSum += (estHoverThr + throttleDeltaUs);  // Use current total throttle
        hoverCount++;
        lastHoverMs = nowMs;
        if (hoverCount >= 200) {  // Converge after ~10 s (200 samples @50ms)
          float avgThr = thrSum / hoverCount;
          estHoverThr = lpfStep(estHoverThr, avgThr, HOVER_EST_ALPHA);  // Slow update
          Serial.printf("[HOVER] Est. throttle: %.0f µs (from %d samples)\n", estHoverThr, hoverCount);
          hoverCount = 0; thrSum = 0.0f;
        }
      }
    } else {
      hoverCount = 0; thrSum = 0.0f;  // Reset on motion
    }

    // Build final throttle around estimated hover
    int throttleUs = (int)lround(estHoverThr + throttleDeltaUs);
    throttleUs = clampi(throttleUs, 1000, 2000);

    // ---- Motor mixing ----
    int m1 = (int)lround(throttleUs + rollOut + pitchOut - yawOut);
    int m2 = (int)lround(throttleUs - rollOut + pitchOut + yawOut);
    int m3 = (int)lround(throttleUs - rollOut - pitchOut - yawOut);
    int m4 = (int)lround(throttleUs + rollOut - pitchOut + yawOut);
    m1 = clampi(m1, 1000, 2000);
    m2 = clampi(m2, 1000, 2000);
    m3 = clampi(m3, 1000, 2000);
    m4 = clampi(m4, 1000, 2000);

    // ---- Safety: disarmed/RC invalid → cut + reset integrators ----
    if (vzStick < 1050 || !rc.valid) {
      m1 = m2 = m3 = m4 = 1000;
      integralRoll = integralPitch = integralYaw = 0;
      prevErrorRoll = prevErrorPitch = prevErrorYaw = 0;
      integralAngleRoll = integralAnglePitch = 0;
      prevErrorAngleRoll = prevErrorAnglePitch = 0;
      integralVz = 0;
      prevErrorVz = 0;
      vz_est = 0.0f; // Reset for clean restart
      // Reset estimation on disarm
      estHoverThr = 1500.0f;  // Back to initial guess
      hoverCount = 0; thrSum = 0.0f;
    }

    // ---- Output + publish ----
    MotorFeed out{m1, m2, m3, m4};
    if (xSemaphoreTake(gMutexMotors, pdMS_TO_TICKS(1)) == pdTRUE) { gMotors = out; xSemaphoreGive(gMutexMotors); }
    ledcWrite(1, usToTicks(m1));
    ledcWrite(2, usToTicks(m2));
    ledcWrite(3, usToTicks(m3));
    ledcWrite(4, usToTicks(m4));
  }
}

void startTaskControl() {
  xTaskCreatePinnedToCore(taskControl, "CTRL", 4096, nullptr, PRIO_CTRL, &sTaskHandle, APP_CPU_NUM);
}