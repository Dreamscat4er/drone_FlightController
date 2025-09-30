#include "Shared.h"

// Queues & mutexes
QueueHandle_t gQPPM = nullptr;
SemaphoreHandle_t gMutexAngles = nullptr;
SemaphoreHandle_t gMutexRC = nullptr;

// Shared states
RCState gRC = {{1500,1500,1000,1500,0,0,0,0}, 0, false};
AttState gAtt = {0,0,0,0,0,0};

// PID state
float prevErrorRoll = 0, prevErrorPitch = 0, prevErrorYaw = 0;
float integralRoll = 0, integralPitch = 0, integralYaw = 0;
float prevErrorAngleRoll = 0, prevErrorAnglePitch = 0;
float integralAngleRoll = 0, integralAnglePitch = 0;

// PID gains
float Kp_angle = 6.5f, Ki_angle = 0.0f, Kd_angle = 0.0f;
float Kp_rate  = 0.15f, Ki_rate  = 0.1f, Kd_rate  = 0.003f;
float Kp_yaw   = 0.2f,  Ki_yaw   = 0.05f, Kd_yaw  = 0.0f;

// Gyro bias
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
