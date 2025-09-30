/*
  TaskControl.cpp
  ----------------
  This file implements the **control loop task** for the quadcopter.
  Responsibilities:
   - Reads the latest RC state (commands) and IMU state (angles, rates)
   - Runs the cascaded PID controllers (Angle → Rate)
   - Mixes control outputs into motor commands (m1–m4)
   - Updates PWM outputs (LEDC channels)
   - Enforces failsafe: cuts motors if RC signal is invalid or throttle is low
*/

#include "TaskControl.h"
#include "Utils.h"
#include "Config.h"

// Handle for FreeRTOS task
static TaskHandle_t sTaskHandle = nullptr;

// === Control task ===
static void taskControl(void* arg) {
  // Setup PWM hardware (LEDC on ESP32)
  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(3, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(4, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(PWM1_PIN, 1);
  ledcAttachPin(PWM2_PIN, 2);
  ledcAttachPin(PWM3_PIN, 3);
  ledcAttachPin(PWM4_PIN, 4);

  const TickType_t period = pdMS_TO_TICKS(CTRL_PERIOD_MS); // Control loop period (e.g. 5 ms = 200 Hz)
  TickType_t last = xTaskGetTickCount();                   // For precise periodic timing

  for (;;) {
    vTaskDelayUntil(&last, period);  // Run task at fixed frequency

    // === Copy shared state ===
    RCState rc;
    AttState att;

    // Copy RC channels (protected by mutex)
    if (xSemaphoreTake(gMutexRC, pdMS_TO_TICKS(1)) == pdTRUE) {
      rc = gRC;
      xSemaphoreGive(gMutexRC);
    } else continue; // Skip iteration if lock failed

    // Copy attitude state (protected by mutex)
    if (xSemaphoreTake(gMutexAngles, pdMS_TO_TICKS(1)) == pdTRUE) {
      att = gAtt;
      xSemaphoreGive(gMutexAngles);
    } else continue;

    // === Map and clean RC inputs ===
    float targetRoll    = map(rc.ch[0], 1000, 2000, -MAX_ANGLE, MAX_ANGLE);
    float targetPitch   = map(rc.ch[1], 1000, 2000, -MAX_ANGLE, MAX_ANGLE);
    float throttle      = rc.ch[2];
    float targetYawRate = map(rc.ch[3], 1000, 2000, -MAX_YAW_RATE, MAX_YAW_RATE);

    // Apply small deadband to avoid jitter
    targetRoll    = applyDeadband(targetRoll, 1.0f);
    targetPitch   = applyDeadband(targetPitch, 1.0f);
    targetYawRate = applyDeadband(targetYawRate, 1.0f);

    // Failsafe: force throttle low if RC is invalid
    if (!rc.valid) throttle = 1000;

    const float dt = CTRL_PERIOD_MS / 1000.0f; // Loop time in seconds

    // === Outer loop: Angle → desired rate ===
    float desiredRateRoll  = computePID(targetRoll,  att.angleRoll,  integralAngleRoll,  prevErrorAngleRoll,  Kp_angle, Ki_angle, Kd_angle, dt);
    float desiredRatePitch = computePID(targetPitch, att.anglePitch, integralAnglePitch, prevErrorAnglePitch, Kp_angle, Ki_angle, Kd_angle, dt);

    // === Inner loop: Rate → motor correction ===
    float rollOut  = computePID(desiredRateRoll,  att.rateRoll,  integralRoll,  prevErrorRoll,  Kp_rate, Ki_rate, Kd_rate, dt);
    float pitchOut = computePID(desiredRatePitch, att.ratePitch, integralPitch, prevErrorPitch, Kp_rate, Ki_rate, Kd_rate, dt);
    float yawOut   = computePID(targetYawRate,     att.rateYaw,  integralYaw,  prevErrorYaw,   Kp_yaw,  Ki_yaw,  Kd_yaw,  dt);

    // === Motor mixing ===
    throttle = (throttle > 1800) ? 1800 : throttle; // Clamp throttle for safety
    int m1 = (int)lround(throttle + rollOut + pitchOut - yawOut);
    int m2 = (int)lround(throttle - rollOut + pitchOut + yawOut);
    int m3 = (int)lround(throttle - rollOut - pitchOut - yawOut);
    int m4 = (int)lround(throttle + rollOut - pitchOut + yawOut);

    // Limit outputs to ESC range (1000–2000 µs)
    m1 = constrain(m1, 1000, 2000);
    m2 = constrain(m2, 1000, 2000);
    m3 = constrain(m3, 1000, 2000);
    m4 = constrain(m4, 1000, 2000);

    // Cut motors if disarmed (low throttle or invalid RC)
    if (throttle < 1050 || !rc.valid) {
      m1 = m2 = m3 = m4 = 1000;
      // Reset PID integrators to avoid windup when disarmed
      integralRoll = integralPitch = integralYaw = 0;
      prevErrorRoll = prevErrorPitch = prevErrorYaw = 0;
    }

    // === Write to motors ===
    ledcWrite(1, usToTicks(m1));
    ledcWrite(2, usToTicks(m2));
    ledcWrite(3, usToTicks(m3));
    ledcWrite(4, usToTicks(m4));
  }
}

// === Task launcher ===
void startTaskControl() {
  xTaskCreatePinnedToCore(taskControl, "CTRL", 4096, nullptr, PRIO_CTRL, &sTaskHandle, APP_CPU_NUM);
}
