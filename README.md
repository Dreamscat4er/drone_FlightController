This is a real-time flight controller for a brushed/brushless quad built on ESP32 + FreeRTOS and made for Savonia UAS for educational purposes by [Stanislav Kolosov]([https://github.com/yourusername](https://github.com/Dreamscat4er)
It reads an MPU6050 IMU and a BMP280 barometer, accepts PPM RC input, runs attitude and yaw control loops, and uses a tiny EKF to estimate vertical velocity from IMU + baro. Throttle is managed around a slowly adapting hover estimate so mid-stick hovers without a fixed “magic number”.

Electric schema
<img width="1097" height="831" alt="image" src="https://github.com/user-attachments/assets/0f85ddd5-65db-4d1e-a628-d790ffcbe023" />
