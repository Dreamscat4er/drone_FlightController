This is a real-time flight controller for a brushed/brushless quad built on ESP32 + FreeRTOS.
It reads an MPU6050 IMU and a BMP280 barometer, accepts PPM RC input, runs attitude, altitude, and yaw control loops, and uses a tiny EKF (Extended Kalman Filter) to estimate vertical velocity from IMU + barometer. 

Electric schema
<img width="1097" height="831" alt="image" src="https://github.com/user-attachments/assets/0f85ddd5-65db-4d1e-a628-d790ffcbe023" />

RTOS Tasks overview.

The flight controller operates on an ESP32 microcontroller, using FreeRTOS for multitasking. Each task is a lightweight thread pinned to a specific core (PRO_CPU_NUM for sensors, APP_CPU_NUM for control/RC). Tasks are prioritized to ensure critical operations preempt non-essential ones.

Tasks:

TaskRC: Processes RC receiver signals.
TaskIMU: Handles IMU (MPU6050) readings and attitude computation.
TaskBaro: Manages barometer (BMP280) for altitude sensing.
TaskControl: Performs EKF fusion, PID control, and motor mixing.
TaskLog: Outputs telemetry for monitoring.

**TaskRC** (Priority: PRIO_RC, Core: APP):

Function: Captures PPM pulses via ISR (handlePPMInterrupt) when the pilot touches the RC sticks, then queues them (gQPPM), and assembles them into RCState gRC (8 channels, valid flag).
Connections: 
ISR → Queue → TaskRC → gMutexRC → gRC (consumed by TaskControl).


**TaskIMU** (Priority: PRIO_IMU, Core: PRO):

Function: Initializes/calibrates MPU6050, reads accel/gyro data, applies biases/inversion, computes complementary-filter attitude (angleRoll/Pitch), and linear Z-accel (aZlinLP).
Connections: 
I2C (mutex gMutexI2C) → Local compute → gMutexAngles → gAtt; 
gMutexVertAcc → gVertAcc (feeds TaskControl EKF).


**TaskBaro** (Priority: PRIO_BARO, Core: PRO):

Function: Initializes/calibrates BMP280, reads temp/pressure/altitude, zeros relative to startup, applies LPF (sAltLP).
Connections: 
I2C (gMutexI2C) → Compute → gMutexBaro → gBaro (feeds TaskControl EKF updates).


**TaskControl** (Priority: PRIO_CTRL, Core: APP):

Function: Orchestrates flight: Maps RC to targets; runs attitude PIDs; fuses EKF (predict/update on gVertAcc.aZlin and gBaro.altitude); velocity PID on vz_est; dynamic hover estimation; motor mixing/PWM output.
Connections: 
Consumes gMutexRC → gRC; 
gMutexAngles → gAtt; 
gMutexBaro → gBaro; 
gMutexVertAcc → gVertAcc. Produces gMutexMotors → gMotors (read by TaskLog).

Vertical velocity control diagram.
<img width="1199" height="657" alt="image" src="https://github.com/user-attachments/assets/4d8b659e-71a7-49bb-ac65-61423bf99ce8" />

Cascaded attitude control diagram. 
<img width="1512" height="693" alt="image" src="https://github.com/user-attachments/assets/2914ab6a-a884-495e-bfda-37c72a903f52" />
Outer Loop: Computes desired angular rates (deg/s) from angle errors.Input error = targetRoll/Pitch (pilot command, µs pulses mapped to deg). Output is desiredRateRoll/Pitch (target angular velocity, deg/s)—what the drone should rotate at to reach the angle.
Inner Loop: Computes torque commands (motor pulses in µs) from rate errors. Input: Error = desiredRateRoll/Pitch (from outer loop). Output aretorque deltas (µs) — added to throttle in motor mixing.



**TaskLog** (Priority: PRIO_LOG, Core: PRO):

Function: Telemetry output: Snaps and prints RC/attitude/rates/motors to Serial.
Connections: 
Reads gMutexRC → gRC; g
MutexAngles → gAtt; 
gMutexMotors → gMotors.

Task diagram.
<img width="1506" height="766" alt="image" src="https://github.com/user-attachments/assets/0091d176-61ca-45e2-9d97-a4d488f320dc" />


**Links:**
1. [ESP32](https://mischianti.org/vcc-gnd-studio-yd-esp32-s3-devkitc-1-clone-high-resolution-pinout-and-specs/)
2. [MPU6050](https://learn.adafruit.com/mpu6050-6-dof-accelerometer-and-gyro/downloads)
3. [BMP280](https://learn.adafruit.com/adafruit-bmp280-barometric-pressure-plus-temperature-sensor-breakout/downloads)

**Documentation (Doxygen)**

This project uses Doxygen to generate browsable HTML documentation from the code comments.

1) Install Doxygen
Windows (recommended: installer)

Download the Windows installer from the official site and run it.

```powershell
doxygen --version
```
macOS (Homebrew)

```bash
brew install doxygen graphviz
doxygen --version
```

Ubuntu/Debian

```bash
sudo apt update
sudo apt install doxygen graphviz
doxygen --version
```
2) Open index.html in docs/html

**Credits:**

Made for Savonia UAS for educational purposes by [Stanislav Kolosov](https://github.com/Dreamscat4er)
