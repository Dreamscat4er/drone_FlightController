#include "Shared.h"

// === Queues & mutexes ===
QueueHandle_t     gQPPM         = nullptr;
SemaphoreHandle_t gMutexAngles  = nullptr;
SemaphoreHandle_t gMutexRC      = nullptr;
SemaphoreHandle_t gMutexMotors  = nullptr;
SemaphoreHandle_t gMutexBaro    = nullptr;
SemaphoreHandle_t gMutexI2C     = nullptr;
SemaphoreHandle_t gMutexVertAcc = nullptr; 

// === Shared states ===
RCState   gRC     = {{1500,1500,1000,1500,0,0,0,0}, 0, false};
AttState  gAtt    = {0,0,0,0,0,0};
BaroState gBaro   = {0,0,0,0,false}; 
MotorFeed gMotors = {0,0,0,0};

// world-frame, gravity-removed vertical acceleration (producer: TaskIMU)
VertAccState gVertAcc = {0.0f, 0, false};    // aZlin, stampUs, valid

// === PID state ===
float prevErrorRoll = 0, prevErrorPitch = 0, prevErrorYaw = 0;
float integralRoll = 0, integralPitch = 0, integralYaw = 0;
float prevErrorAngleRoll = 0, prevErrorAnglePitch = 0;
float integralAngleRoll = 0, integralAnglePitch = 0;
float prevErrorVz = 0, integralVz  = 0;

// === PID gains ===
// Angle outer loop (deg → deg/s ref)
float Kp_angle = 6.5f, Ki_angle = 0.0f, Kd_angle = 0.0f;
// Rate inner loop (deg/s error → motor µs)
float Kp_rate  = 0.15f, Ki_rate  = 0.10f, Kd_rate  = 0.003f;
// Yaw rate loop
float Kp_yaw   = 0.20f,  Ki_yaw   = 0.05f, Kd_yaw  = 0.00f;
// Altitude-hold outer loop (alt error → Vz command)
float Kp_alt   = 1.5f,  Ki_alt   = 0.3f,  Kd_alt  = 0.0f;
// Vertical rate (Vz error → throttle µs)
float Kp_vz    = 1.5f, Ki_vz   = 0.1f, Kd_vz   = 0.3f;

