#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// === Pin Configuration ===
#define SDA_PIN 5
#define SCL_PIN 6
#define PPM_PIN 4
#define PWM1_PIN 13 // m1 (cw)
#define PWM2_PIN 10 // m2 (ccw)
#define PWM3_PIN 11 // m3 (cw)
#define PWM4_PIN 12 // m4 (ccw)

// === Constants ===
#define CHANNELS 8
#define PPM_EOF 3000
#define PWM_FREQ 50 
#define PWM_RESOLUTION 10
#define STANDART_PWM_RESOLUTION 16    
#define REFRESH_PERIOD 20000
#define PWM_LEVELS (1 << PWM_RESOLUTION)
#define MAX_ANGLE 30.0
#define MAX_YAW_RATE 75.0

// === Global Variables ===
Adafruit_MPU6050 mpu;
volatile uint16_t ppmValues[CHANNELS];
volatile uint32_t lastTime = 0;
volatile uint8_t currentPulse = 0;

float angleRoll, anglePitch;
float rateRoll, ratePitch, rateYaw;

// PID State
float prevErrorRoll = 0, prevErrorPitch = 0, prevErrorYaw = 0;
float integralRoll = 0, integralPitch = 0, integralYaw = 0;
float prevErrorAngleRoll = 0, prevErrorAnglePitch = 0;
float integralAngleRoll = 0, integralAnglePitch = 0;

// PID Gains
float Kp_angle = 6.5, Ki_angle = 0.0, Kd_angle = 0.0;
float Kp_rate = 0.15, Ki_rate = 0.1, Kd_rate = 0.003;
float Kp_yaw = 0.2, Ki_yaw = 0.05, Kd_yaw = 0.0;

uint32_t loopTimer;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

// === Utility Functions ===
float applyDeadband(float value, float threshold) {
  return (abs(value) < threshold) ? 0.0 : value;
}

float constrainAndAntiWindup(float value, float &integral, float minVal, float maxVal) {
  if (value > maxVal || value < minVal) {
    integral = 0; // prevent windup
  }
  return constrain(value, minVal, maxVal);
}

// === Interrupt: PPM Reading ===
void IRAM_ATTR handlePPMInterrupt() {
  uint32_t now = micros();
  uint32_t pulse = now - lastTime;
  lastTime = now;
  if (pulse > PPM_EOF) {
    currentPulse = 0;
  } else if (currentPulse < CHANNELS) {
    ppmValues[currentPulse] = pulse;
    currentPulse++;
  }
}

// === Complementary Filter with Gyro Bias Correction ===
void computeAngles() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accelX = a.acceleration.x;
  float accelY = a.acceleration.y;
  float accelZ = a.acceleration.z;

  float gyroX = applyDeadband((g.gyro.x - gyroBiasX) * RAD_TO_DEG, 0.5);
  float gyroY = applyDeadband((g.gyro.y - gyroBiasY) * RAD_TO_DEG, 0.5);
  float gyroZ = applyDeadband((g.gyro.z - gyroBiasZ) * RAD_TO_DEG, 0.5);
  

  float pitchAcc = applyDeadband(atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * RAD_TO_DEG, 5);
  float rollAcc  = applyDeadband(atan2(accelY, accelZ) * RAD_TO_DEG, 5);

  anglePitch = (0.98 * (anglePitch + gyroY * 0.02) + 0.02 * pitchAcc);
  angleRoll  = (0.98 * (angleRoll  + gyroX * 0.02) + 0.02 * rollAcc);

  rateRoll = gyroX;
  ratePitch = gyroY;
  rateYaw = gyroZ;

  // Serial.print("Gyro (deg/s) → ");
  // Serial.print("X: "); Serial.print(rateRoll, 2); Serial.print("  ");
  // Serial.print("Y: "); Serial.print(ratePitch, 2); Serial.print("  ");
  // Serial.print("Z: "); Serial.println(rateYaw, 2);

  // Serial.print("Angles → ");
  // Serial.print("Roll: "); Serial.print(angleRoll, 2); Serial.print("°  ");
  // Serial.print("Pitch: "); Serial.print(anglePitch, 2); Serial.println("°");

}

// === Generic PID Function with Deadband and Anti-Windup ===
float computePID(float target, float current, float &integral, float &prev_error,
                 float Kp, float Ki, float Kd, float dt = 0.02,
                 float outputMin = -500, float outputMax = 500,
                 float errorDeadband = 2) {

  float error = target - current;
  error = applyDeadband(error, errorDeadband);
  if (error == 0) {
    integral *= 0.95;
  }
  
  integral += error * dt;
  float derivative = (error - prev_error) / dt;
  prev_error = error;

  float output = Kp * error + Ki * integral + Kd * derivative;
  return constrainAndAntiWindup(output, integral, outputMin, outputMax);
}

uint16_t usToTicks(uint16_t usWidth) {
 float result = ((float)usWidth / ((float)REFRESH_PERIOD / (float)PWM_LEVELS)); // turnicating to int  
 //if (result < 52) result = 52; // minimum value for the motor to start
 //if (result > 102) result = 102; // maximum value for the motor to start

 return (int) round(result); 
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n🚀 Starting drone setup...");

  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("✅ I2C initialized");

  if (!mpu.begin()) {
    Serial.println("❌ MPU6050 not found!");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // === Gyro Bias Calibration ===
  Serial.println("🧭 Calibrating gyroscope...");
  for (int i = 0; i < 2000; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroBiasX += g.gyro.x;
    gyroBiasY += g.gyro.y;
    gyroBiasZ += g.gyro.z;

    
  }
  gyroBiasX /= 2000.0;
  gyroBiasY /= 2000.0;
  gyroBiasZ /= 2000.0;
 
  Serial.println("✅ Gyro bias calibrated");

  pinMode(PPM_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), handlePPMInterrupt, RISING);

  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);
  pinMode(PWM3_PIN, OUTPUT);
  pinMode(PWM4_PIN, OUTPUT);

  ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(2, PWM_FREQ, PWM_RESOLUTION); 
  ledcSetup(3, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(4, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(PWM1_PIN, 1);
  ledcAttachPin(PWM2_PIN, 2);
  ledcAttachPin(PWM3_PIN, 3);
  ledcAttachPin(PWM4_PIN, 4);
  // analogWriteFrequency(PWM1_PIN, PWM_FREQ);
  // analogWriteFrequency(PWM2_PIN, PWM_FREQ);
  // analogWriteFrequency(PWM3_PIN, PWM_FREQ);
  // analogWriteFrequency(PWM4_PIN, PWM_FREQ);


  //loopTimer = micros();
  Serial.println("✅ Setup complete");
}

// === Main Loop ===
void loop() {
  //float dt = (micros() - loopTimer) / 1000000.0;
  loopTimer = micros();

  computeAngles();

  // Read and map RC inputs
  float targetRoll = map(ppmValues[0], 1000, 2000, -MAX_ANGLE, MAX_ANGLE);
  float targetPitch = map(ppmValues[1], 1000, 2000, -MAX_ANGLE, MAX_ANGLE);
  float targetYawRate = map(ppmValues[3], 1000, 2000, -MAX_YAW_RATE, MAX_YAW_RATE);
  float throttle = ppmValues[2];

  // Serial.print("Mapped Inputs → ");
  // Serial.print("Roll: "); Serial.print(targetRoll, 2); Serial.print("°  ");
  // Serial.print("Pitch: "); Serial.print(targetPitch, 2); Serial.print("°  ");
  // Serial.print("YawRate: "); Serial.print(targetYawRate, 2); Serial.print("°/s  ");
  // Serial.print("Throttle: "); Serial.println(throttle, 0);

  // Apply deadbands to eliminate jitter
  targetRoll = applyDeadband(targetRoll, 1.0);
  targetPitch = applyDeadband(targetPitch, 1.0);
  targetYawRate = applyDeadband(targetYawRate, 1.0);

  float desiredRateRoll = computePID(targetRoll, angleRoll, integralAngleRoll, prevErrorAngleRoll, Kp_angle, Ki_angle, Kd_angle);
  float desiredRatePitch = computePID(targetPitch, anglePitch, integralAnglePitch, prevErrorAnglePitch, Kp_angle, Ki_angle, Kd_angle);

  float rollOutput = computePID(desiredRateRoll, rateRoll, integralRoll, prevErrorRoll, Kp_rate, Ki_rate, Kd_rate);
  float pitchOutput = computePID(desiredRatePitch, ratePitch, integralPitch, prevErrorPitch, Kp_rate, Ki_rate, Kd_rate);
  float yawOutput = computePID(targetYawRate, rateYaw, integralYaw, prevErrorYaw, Kp_yaw, Ki_yaw, Kd_yaw);
  
  throttle = (throttle > 1800) ? 1800 : throttle; // Limit throttle to 1800 for safety
  int m1 = (int)round(throttle + rollOutput + pitchOutput - yawOutput);
  int m2 = (int)round(throttle - rollOutput + pitchOutput + yawOutput);
  int m3 = (int)round(throttle - rollOutput - pitchOutput - yawOutput);
  int m4 = (int)round(throttle + rollOutput - pitchOutput + yawOutput);

  m1 = constrain(m1, 1000, 2000);
  m2 = constrain(m2, 1000, 2000);
  m3 = constrain(m3, 1000, 2000);
  m4 = constrain(m4, 1000, 2000);

  if (throttle < 1050) {
    m1 = m2 = m3 = m4 = 1000;
    integralRoll = integralPitch = integralYaw = 0;
    prevErrorRoll = prevErrorPitch = prevErrorYaw = 0;
  }



  ledcWrite(1, usToTicks(m1));
  ledcWrite(2, usToTicks(m2));
  ledcWrite(3, usToTicks(m3));
  ledcWrite(4, usToTicks(m4));

  // analogWrite(PWM1_PIN, usToTicks(m1));
  // analogWrite(PWM2_PIN, usToTicks(m2));
  // analogWrite(PWM3_PIN, usToTicks(m3));
  // analogWrite(PWM4_PIN, usToTicks(m4));
  

  Serial.print("PWM Ticks → ");
  Serial.print("M1: "); Serial.print(usToTicks(m1)); Serial.print("  ");
  Serial.print("M2: "); Serial.print(usToTicks(m2)); Serial.print("  ");
  Serial.print("M3: "); Serial.print(usToTicks(m3)); Serial.print("  ");
  Serial.print("M4: "); Serial.println(usToTicks(m4));



  // Serial.print("Motor Outputs: ");
  // Serial.print("M1: "); Serial.print(m1); Serial.print("  ");
  // Serial.print("M2: "); Serial.print(m2); Serial.print("  ");
  // Serial.print("M3: "); Serial.print(m3); Serial.print("  ");
  // Serial.print("M4: "); Serial.println(m4);
  while (micros() - loopTimer < 20000); // Maintain 50 Hz loop rate
}
