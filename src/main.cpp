#include <Arduino.h>
#include <Wire.h>
#include <esp32-hal-ledc.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>



//Gyro
// #define MPU6050_ADDR 0x68
// #define CONFIG_REGISTER_1 0x1A //LPF config
// #define CONFIG_REGISTER_2 0x1B //sensitivity config
// #define DLPF_CFG 0x05 // 10Hz LPF
// #define FS_SEL 0x08 // +/- 500 deg/s
#define SDA_PIN 5
#define SCL_PIN 6
//PPM
#define PPM_PIN 4            // The GPIO pin connected to the PPM signal
#define CHANNELS 8  
#define PPM_EOF 3000         // Number of channels in the PPM signal
#define PWM1_PIN 9           // GPIO pin connected to ESC throttle input
#define PWM2_PIN 10
#define PWM3_PIN 11
#define PWM4_PIN 12      
#define PWM_FREQ 50     // Frequency of PWM signal in Hz (50 Hz)
#define REFRESH_PERIOD 20000 //pulse cycle
#define PWM_RESOLUTION 10 // timer width
#define PWM_LEVELS ((1 << PWM_RESOLUTION) - 1)//maximum possible number of ticks based on the timer width
//PID Control
#define ROTATION_RATE 0.15 // from -75deg/s to 75deg/s (150deg/1000us)
//GYRO
float RateRoll, RatePitch, RateYaw;
float angePitch, angleRoll;
float CalibrationRoll, CalibrationPitch, CalibrationYaw;
int SamplesNumber;
#define RAD_TO_DEG  (180.0 / 3.141592653589793) 
//PPM
volatile uint16_t ppmValues[CHANNELS];  // Array to store channel values
volatile uint32_t lastTime = 0;         // Last time a pulse was detected
volatile uint8_t currentPulse = 0;      // Current pulse index being processed

//PID Control
uint32_t loopTimer; // PID Control loop timer
float DesiredRoll, DesiredPitch, DesiredYaw;
float ErrorRoll, ErrorPitch, ErrorYaw;
float InputThrottle, InputRoll, InputPitch, InputYaw;
float PrevErrorRoll, PrevErrorPitch, PrevErrorYaw;
float PrevItermRoll, PrevItermPitch, PrevItermYaw;
float PID_Return[] = {0, 0, 0};
float P_TermRoll = 0.6; float P_TermPitch = P_TermRoll; float P_TermYaw = 2;
float I_TermRoll = 3.5; float I_TermPitch = I_TermRoll; float I_TermYaw = 12;
float D_TermRoll = 0.03; float D_TermPitch = D_TermRoll; float D_TermYaw = 0;

float P_TermAngle = 4.5; // Proportional gain for angle control
float I_TermAngle = 0.1; // Integral gain for angle control
float D_TermAngle = 0.05; // Derivative gain for angle control

float ErrorAngleRoll, ErrorAnglePitch; // Angle errors
float PrevErrorAngleRoll = 0, PrevErrorAnglePitch = 0;
float PrevItermAngleRoll = 0, PrevItermAnglePitch = 0;
float InputRollAngle = 0, InputPitchAngle = 0;




float Motor1_Input, Motor2_Input, Motor3_Input, Motor4_Input;
Adafruit_MPU6050 mpu;

// PPM pulse length calculation
void IRAM_ATTR handlePPMInterrupt() {
    uint32_t timeNow = micros();
    uint32_t pulseLength = timeNow - lastTime; // Calculate pulse length
    lastTime = timeNow; // Store the current time

    if (pulseLength > PPM_EOF) { // Check if this pulse is the end of a frame
        currentPulse = 0; // Reset pulse index to start new frame
    } else {
        if (currentPulse < CHANNELS) { 
            ppmValues[currentPulse] = pulseLength; // Store pulse length for current channel
        }
        currentPulse++; // Move to the next pulse
    }
}


 void gyroReadings() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  RateRoll = g.gyro.x * RAD_TO_DEG;
  RatePitch = g.gyro.y * RAD_TO_DEG;
  RateYaw = g.gyro.z * RAD_TO_DEG;

 }

 // Function to calculate angles from accelerometer and gyro
void calculateAngles(float &pitch, float &roll) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Get accelerometer data
  float accelX = a.acceleration.x; // Forward/backward (X axis)
  float accelY = a.acceleration.y; // Side-to-side (Y axis)
  float accelZ = a.acceleration.z; // Vertical (Z axis)

  // Define pitch as forward/backward tilt (using X and Z axes)
  pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * RAD_TO_DEG;

  // Define roll as side-to-side tilt (using Y and Z axes)
  roll = atan2(-accelX, accelZ) * RAD_TO_DEG;

  // Complementary filter for stability (using gyro data for rate of change)
  float gyroX = g.gyro.x; // Roll rate (side-to-side)
  float gyroY = g.gyro.y; // Pitch rate (forward/backward)

  // Apply complementary filter: combine accelerometer and gyroscope data
  pitch = 0.98 * (pitch + gyroY * 0.02) + 0.02 * pitch;
  roll = 0.98 * (roll + gyroX * 0.02) + 0.02 * roll;
}

//PID Function
void PID_Function (float Error, float P, float I, float D, float PrevError, float PrevIterm ){
  //if (Error)
    float Pterm = P * Error;
    float Iterm = PrevIterm + I * (Error + PrevError) * 0.02 / 2; // remove + PrevError

    

    // Clamp the Iterm
    if (Iterm > 400) { Iterm = 400; }
    else if (Iterm < -400) { Iterm = -400; }

    float Dterm = D * (Error - PrevError) / 0.02;
    float PID_Output = Pterm + Iterm + Dterm;

    // Clamp the PID output
    if (PID_Output > 400) { PID_Output = 400; }
    else if (PID_Output < -400) { PID_Output = -400; }

    PID_Return[0] = PID_Output;
    PID_Return[1] = Error;
    PID_Return[2] = Iterm;
}

void reset_PID () {
  PrevErrorRoll = 0; PrevErrorPitch = 0; PrevErrorYaw = 0;
  PrevItermRoll = 0; PrevItermPitch = 0; PrevItermYaw = 0;
  PrevErrorAnglePitch = 0; PrevErrorAngleRoll = 0;
}

uint16_t usToTicks(uint16_t usWidth) {
    
    return (int)((float)usWidth / ((float)REFRESH_PERIOD / (float)PWM_LEVELS) * (((float)PWM_FREQ) / 50.0)); 
}


void setup() {
  Serial.begin(115200);
  InputRoll = 0;
  InputPitch = 0;
  InputYaw = 0;
  PID_Return[0] = 0;
  PID_Return[1] = 0;
  PID_Return[2] = 0;
         
  Wire.begin(SDA_PIN, SCL_PIN);         
  

  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  

  pinMode(PPM_PIN, INPUT);  
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);
  pinMode(PWM3_PIN, OUTPUT);
  pinMode(PWM4_PIN, OUTPUT);
  
 
 ledcAttachChannel(PWM1_PIN, PWM_FREQ, PWM_RESOLUTION, 0);
 ledcAttachChannel(PWM2_PIN, PWM_FREQ, PWM_RESOLUTION, 1);
 ledcAttachChannel(PWM3_PIN, PWM_FREQ, PWM_RESOLUTION, 2);
 ledcAttachChannel(PWM4_PIN, PWM_FREQ, PWM_RESOLUTION, 3);
 
 

  // Attach an interrupt to the PPM pin to detect changes
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), handlePPMInterrupt, RISING);

  loopTimer = micros();
  for (int i = 0; i <= 2000; i++){
    gyroReadings();
    CalibrationRoll += RateRoll;
    CalibrationPitch += RatePitch;
    CalibrationYaw += RateYaw;

    
  }
  CalibrationRoll /= 2000;
  CalibrationPitch /= 2000;
  CalibrationYaw /= 2000;

 
  
  
}

void loop() {
  calculateAngles(angleRoll, angePitch); //TODO: function gives pitch instead roll. Correct later
  gyroReadings();
  RateRoll -= CalibrationRoll;
  RatePitch -= CalibrationPitch;
  RateYaw -= CalibrationYaw;

  // Serial.print("angleRoll: ");
  // Serial.print(angleRoll);
  // Serial.print(" deg ");

  // Serial.print("angePitch: ");
  // Serial.print(angePitch);
  // Serial.println(" deg ");

    
 
  

  // Print the gyro rates in degrees per second
  // Serial.print("RateRoll: ");
  // Serial.print(RateRoll);
  // Serial.print(" deg/sec, ");

  // Serial.print("RatePitch: ");
  // Serial.print(RatePitch);
  // Serial.print(" deg/sec, ");

  // Serial.print("RateYaw: ");
  // Serial.print(RateYaw);
  // Serial.println(" deg/sec");

  DesiredRoll = ROTATION_RATE*(ppmValues[0]-1500);
  DesiredPitch = ROTATION_RATE*(ppmValues[1]-1500);
  DesiredYaw = ROTATION_RATE*(ppmValues[3]-1500);
  InputThrottle = ppmValues[2];
  
  


// print ppm values
  // Serial.print("Roll: ");
  // Serial.print(ppmValues[0]);
  // Serial.print(" ms, ");

  // Serial.print("Pitch: ");
  // Serial.print(ppmValues[1]);
  // Serial.print(" deg/sec, ");

  // Serial.print("Yaw: ");
  // Serial.print(ppmValues[3]);
  // Serial.println(" deg/sec");
  
  // Serial.print("InputThrottle: ");
  // Serial.print(InputThrottle);
  // Serial.print(" ms, ");


// print desired rates
  // Serial.print("DesiredRoll: ");
  // Serial.print(DesiredRoll);
  // Serial.print(" deg/sec, ");

  // Serial.print("DesiredPitch: ");
  // Serial.print(DesiredPitch);
  // Serial.print(" deg/sec, ");

  // Serial.print("DesiredYaw: ");
  // Serial.print(DesiredYaw);
  // Serial.println(" deg/sec");
  
  // Serial.print("InputThrottle: ");
  // Serial.print(InputThrottle);
  // Serial.print(" ms, ");


  

  ErrorRoll = (DesiredRoll-RateRoll);
  ErrorPitch = (DesiredPitch-RatePitch);
  ErrorYaw = (DesiredYaw-RateYaw);

  ErrorAngleRoll = 0 - angleRoll; // Target angle is 0
  ErrorAnglePitch = 0 - angePitch; // Target angle is 0

  if (abs(ErrorRoll) <= 5) {
    ErrorRoll = 0;
  }
  if (abs(ErrorPitch) <= 5) {
    ErrorPitch = 0;
  }
  if (abs(ErrorYaw) <= 5) {
    ErrorYaw = 0;
  }
  if (abs(ErrorAngleRoll) <= 3) {
    ErrorAngleRoll = 0;
  }
  if (abs(ErrorAnglePitch) <= 3) {
    ErrorAnglePitch = 0;
  }

  // Serial.print("ErrorRoll: ");
  // Serial.print(ErrorRoll);
  // Serial.print(" rad/s ");

  // Serial.print("ErrorPitch: ");
  // Serial.print(ErrorPitch);
  // Serial.print(" rad/s ");

  // Serial.print("ErrorYaw: ");
  // Serial.print(ErrorYaw);
  // Serial.println(" rad/s");



  // Serial.print("ErrorAngleRoll: ");
  // Serial.print(ErrorAngleRoll);
  // Serial.print(" deg/s ");

  // Serial.print("ErrorAnglePitch: ");
  // Serial.print(ErrorAnglePitch);
  // Serial.println(" deg/s ");



  
  //if (ErrorRoll != 0){
    PID_Function(ErrorRoll, P_TermRoll, I_TermRoll, D_TermRoll, PrevErrorRoll, PrevItermRoll);
    InputRoll = PID_Return[0];
    PrevErrorRoll = PID_Return[1];
    PrevItermRoll = PID_Return[2];
  //} else {InputRoll = 0;}

  //if (ErrorPitch != 0){
    PID_Function(ErrorPitch, P_TermPitch, I_TermPitch, D_TermPitch, PrevErrorPitch, PrevItermPitch);
    InputPitch = PID_Return[0];
    PrevErrorPitch = PID_Return[1];
    PrevItermPitch = PID_Return[2];
  //} else {InputPitch = 0;}

  //if (ErrorYaw != 0){
    PID_Function(ErrorYaw, P_TermYaw, I_TermYaw, D_TermYaw, PrevErrorYaw, PrevItermYaw);
    InputYaw = PID_Return[0];
    PrevErrorYaw = PID_Return[1];
    PrevItermYaw = PID_Return[2];
  //} else {InputYaw = 0;}

    PID_Function(ErrorAngleRoll, P_TermAngle, I_TermAngle, D_TermAngle, PrevErrorAngleRoll, PrevItermAngleRoll);
    InputRollAngle = PID_Return[0];
    PrevErrorAngleRoll = PID_Return[1];
    PrevItermAngleRoll = PID_Return[2];

    PID_Function(ErrorAnglePitch, P_TermAngle, I_TermAngle, D_TermAngle, PrevErrorAnglePitch, PrevItermAnglePitch);
    InputPitchAngle = PID_Return[0];
    PrevErrorAnglePitch = PID_Return[1];
    PrevItermAnglePitch = PID_Return[2];
    
    // Serial.print("InputRollAngle: ");
    // Serial.print(InputRollAngle);
    // Serial.print(" deg/s ");

    // Serial.print("InputPitchAngle: ");
    // Serial.print(InputPitchAngle);
    // Serial.println(" deg/s ");
  

  if (InputThrottle > 1800) {InputThrottle = 1800;} //20% reserved for roll, pitch and yaw rotations 

  Motor1_Input = InputThrottle+InputRoll+InputPitch-InputYaw; //-InputRollAngle-InputPitchAngle;
  Motor2_Input = InputThrottle-InputRoll+InputPitch+InputYaw; //+InputRollAngle-InputPitchAngle;
  Motor3_Input = InputThrottle-InputRoll-InputPitch-InputYaw; //+InputRollAngle+InputPitchAngle;
  Motor4_Input = InputThrottle+InputRoll-InputPitch+InputYaw; //-InputRollAngle+InputPitchAngle;
  if (Motor1_Input>2000) {Motor1_Input = 1999;}
  if (Motor2_Input>2000) {Motor2_Input = 1999;}
  if (Motor3_Input>2000) {Motor3_Input = 1999;}
  if (Motor4_Input>2000) {Motor4_Input = 1999;}
  /*int ThrottleIdle = 1180;
  if (Motor1_Input<ThrottleIdle) {Motor1_Input = ThrottleIdle;}
  if (Motor2_Input<ThrottleIdle) {Motor2_Input = ThrottleIdle;}
  if (Motor3_Input<ThrottleIdle) {Motor3_Input = ThrottleIdle;}
  if (Motor4_Input<ThrottleIdle) {Motor4_Input = ThrottleIdle;}
  */
  int ThrottleCutoff = 1000;
  if (ppmValues[2]<1050) {
    Motor1_Input = ThrottleCutoff;
    Motor2_Input = ThrottleCutoff;
    Motor3_Input = ThrottleCutoff;
    Motor4_Input = ThrottleCutoff;
    reset_PID();
  }

  // Serial.print("Motor1_Input: ");
  // Serial.print(Motor1_Input);
  // Serial.print(" ms ");

  // Serial.print("Motor2_Input: ");
  // Serial.print(Motor2_Input);
  // Serial.print(" ms ");

  // Serial.print("Motor3_Input: ");
  // Serial.print(Motor3_Input);
  // Serial.print(" ms");
  
  // Serial.print("Motor4_Input: ");
  // Serial.print(Motor4_Input);
  // Serial.println(" ms, ");
  
  ledcWrite(PWM1_PIN, usToTicks(Motor1_Input));
  ledcWrite(PWM2_PIN, usToTicks(Motor2_Input));
  ledcWrite(PWM3_PIN, usToTicks(Motor3_Input));
  ledcWrite(PWM4_PIN, usToTicks(Motor4_Input));
  
  

  while (micros()-loopTimer<20000)
  {
    
  }
  loopTimer=micros();
  
}


 