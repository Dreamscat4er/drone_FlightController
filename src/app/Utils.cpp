/*
  Utils.{h,cpp}
  -------------
  Helper functions for math and control:
   - Deadband filtering
   - Constrain with anti-windup reset
   - Generic PID computation
   - Conversion from microseconds → LEDC ticks
*/

#include "Utils.h"
#include <math.h>

float applyDeadband(float v, float th) {
  return (fabsf(v) < th) ? 0.0f : v;
}

float constrainAndAntiWindup(float value, float &integral, float minVal, float maxVal) {
  if (value > maxVal || value < minVal) integral = 0;
  return constrain(value, minVal, maxVal);
}

float computePID(float target, float current, float &integral, float &prev_error,
                 float Kp, float Ki, float Kd, float dt,
                 float outMin, float outMax, float errDB) {
  float error = applyDeadband(target - current, errDB);
  if (error == 0) integral *= 0.95f;
  integral += error * dt;
  float derivative = (error - prev_error) / dt;
  prev_error = error;
  float output = Kp*error + Ki*integral + Kd*derivative;
  return constrainAndAntiWindup(output, integral, outMin, outMax);
}

uint16_t usToTicks(uint16_t usWidth) {
  float levels = float(1u << PWM_RESOLUTION);
  float result = (float)usWidth / ((float)REFRESH_PERIOD / levels);
  return (uint16_t)lroundf(result);
}
