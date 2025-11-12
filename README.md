This is a real-time flight controller for a brushless quad built on ESP32 + FreeRTOS.
It reads an MPU6050 IMU and a BMP280 barometer, accepts PPM RC input, runs attitude, altitude, and yaw control loops, and uses a tiny EKF (Extended Kalman Filter) to estimate vertical velocity from IMU + barometer. 

**Electric schema**

<img width="790" height="682" alt="image" src="https://github.com/user-attachments/assets/c9a8a0d0-5ca3-42dc-a21d-dc4311c23978" />

RTOS Tasks overview.

The flight controller operates on an ESP32 microcontroller, using FreeRTOS for multitasking. Each task is a lightweight thread pinned to a specific core (PRO_CPU_NUM for sensors, APP_CPU_NUM for control/RC). Tasks are prioritized to ensure critical operations preempt non-essential ones.

### Tasks

- **TaskRC** *(Priority: `PRIO_RC`, Core: `APP`)* — Processes RC receiver signals.  
- **TaskIMU** *(Priority: `PRIO_IMU`, Core: `PRO`)* — Handles IMU (MPU6050) readings and attitude computation.  
- **TaskBaro** *(Priority: `PRIO_BARO`, Core: `PRO`)* — Manages barometer (BMP280) for altitude sensing.  
- **TaskControl** *(Priority: `PRIO_CTRL`, Core: `APP`)* — Performs EKF fusion, PID control, and motor mixing.  
- **TaskLog** *(Priority: `PRIO_LOG`, Core: `PRO`)* — Outputs telemetry for monitoring.


Task diagram.
<img width="1506" height="766" alt="image" src="https://github.com/user-attachments/assets/0091d176-61ca-45e2-9d97-a4d488f320dc" />

**Attitude PID schema**

<img width="928" height="439" alt="Screenshot 2025-11-12 at 10 05 14" src="https://github.com/user-attachments/assets/0de035de-0cc7-4653-99bf-d7420ad7c819" />

**Vertical velocity PID schema**

<img width="906" height="432" alt="Screenshot 2025-11-12 at 10 07 32" src="https://github.com/user-attachments/assets/ada7d139-2597-418a-82b9-9a879b9f0053" />

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
