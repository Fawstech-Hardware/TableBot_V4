# TableBot_V4 Firmware Documentation

This document outlines the firmware architecture, operation modes, and control logic for the ESP32 microcontroller powering TableBot_V4.

## Overview

The firmware is written in C++ using the Arduino framework via PlatformIO. It is designed to run asynchronously, handling web server requests, sensor polling, and motor PID control in a continuous `loop()`.

## Core Modules

1. **`main.cpp`**
   - The entry point of the application.
   - Initializes all components in `setup()`.
   - The `loop()` continuously runs:
     - Web server handling (`RobotWeb::handle()`).
     - Sensor updates (`sensors.update()`).
     - Command processing.
     - Mode execution (Manual or Auto).
     - Telemetry broadcasting.

2. **`config.h`**
   - The central configuration file.
   - Defines all GPIO pins, robot physical dimensions, motor speeds, PID tuning parameters, and WiFi credentials.

3. **`MotorControl`** (`MotorControl.h` / `.cpp`)
   - Handles low-level PWM signals to the L298N motor driver.
   - Computes movement distances using quadrature encoder ticks.
   - Implements a PID controller that utilizes MPU6050 yaw data to ensure the robot drives in a perfect straight line.

4. **`Sensors`** (`Sensors.h` / `.cpp`)
   - Manages data collection from 4x IR edge sensors, HC-SR04 ultrasonic sensor (polls every 100ms), and the MPU6050 IMU.
   - Integrates gyro Z-axis data to calculate cumulative yaw in degrees.

5. **`ReactiveCleaner`** (`ReactiveCleaner.h` / `.cpp`)
   - The state machine for autonomous table cleaning.
   - **State: FORWARD**
     - Drives blindly forward utilizing gyro PID to stay straight.
   - **State: REVERSING**
     - Triggered when a front edge or obstacle is detected.
     - Reverses exactly 10cm using encoder measurements (with a fallback timer constraint).
     - Monitors rear edge sensors to prevent falling backward.
   - **State: TURNING**
     - Turns approximately 115° using MPU6050 yaw data to ensure random but comprehensive table coverage.
     - Alternates turn direction (left, then right) across iterations.

6. **`RobotWebServer`** (`RobotWebServer.h` / `.cpp`)
   - Sets up the ESP32 as a WiFi Access Point (`SSID: TableBot`).
   - Hosts an embedded HTML/JS/CSS control panel.
   - Provides REST endpoints for commands (`/cmd`) and auto-cleaning (`/auto`).
   - Serves real-time JSON telemetry (`/telemetry`) at 2Hz.

## Operation Modes

### 1. IDLE Mode
The default state on boot. Motors are stopped, and the robot waits for web commands. Telemetry continues to update.

### 2. MANUAL Mode
Triggered by the D-Pad on the web interface.
- Provides direct control (Forward, Backward, Left, Right).
- **Safety Overrides**: If the robot detects a front edge or obstacle while driving forward, or a rear edge while driving backward, it will aggressively apply the brakes and revert to `IDLE` mode to prevent falling off the table.

### 3. AUTO Mode (Reactive Cleaning)
Triggered via the "Start Cleaning" button.
- Enters the `ReactiveCleaner` state machine.
- Bounces around the table randomly, reversing and rotating whenever an edge or obstacle is met.
- Can be interrupted at any time using the STOP button on the web interface.
