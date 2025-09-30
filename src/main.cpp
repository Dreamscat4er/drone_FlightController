/*
  main.cpp
  --------

  === Task Flow ===
  
  1. **TaskRC (priority 2)**
     - ISR captures PPM pulses from the RC receiver.
     - TaskRC assembles channels into an `RCState` object.
     - Publishes latest RC commands (throttle, roll, pitch, yaw) → `gRC`.

  2. **TaskIMU (priority 3)**
     - Reads MPU6050 over I2C at ~200 Hz.
     - Runs complementary filter to compute roll/pitch angles.
     - Publishes sensor fusion output (angles + angular rates) → `gAtt`.

  3. **TaskControl (priority 4, highest)**
     - Runs control loop at ~200 Hz.
     - Reads `gRC` (user input) and `gAtt` (sensor state).
     - Cascaded PID:  
         Outer loop (Angle PID → desired rates)
            - Input: pilot’s stick command (target roll/pitch angle).
            - Compares desired angle (from RC) vs. measured angle (from IMU).
            - Output: a desired angular rate (deg/s).
            - Example: Pilot wants to tilt to 10° roll and hold it steady. 
         Inner loop (Rate PID → motor corrections)
            - Input: desired rate (from outer loop) vs. measured gyro rate.
            - Output: motor correction values.
            - Example: Motor power changes to reach the desired rate from outer loop
     - Mixes corrections into 4 motor commands (m1–m4).
     - Updates ESCs via LEDC PWM channels.

  4. **TaskLog (priority 1, lowest)**
     - Periodically (10 Hz) prints RC + attitude states to Serial.
     - Purely for debugging/monitoring; no effect on control.

  === Data Synchronization ===
  - `gRC` and `gAtt` are global states protected by FreeRTOS mutexes.
  - RC ISR → queue → TaskRC → `gRC`.
  - TaskIMU writes → `gAtt`.
  - TaskControl reads both states to compute outputs.
  - TaskLog reads both states to print diagnostics.

*/
#ifndef IMU_ONLY_TEST

#include <Arduino.h>
#include "app/Shared.h"
#include "app/Config.h"
#include "app/TaskRC.h"
#include "app/TaskIMU.h"
#include "app/TaskControl.h"
#include "app/TaskLog.h"

void setup() {
  Serial.begin(115200);
  delay(200);

  // Create mutexes/queues
  gMutexAngles = xSemaphoreCreateMutex();
  gMutexRC     = xSemaphoreCreateMutex();
  gQPPM        = xQueueCreate(64, sizeof(PpmPulse));

  // Start tasks
  startTaskRC();
  startTaskIMU();
  startTaskControl();
  startTaskLog();

  Serial.println("FreeRTOS tasks started.");
}

void loop() {
  // Empty: all work is in tasks
}

#endif
