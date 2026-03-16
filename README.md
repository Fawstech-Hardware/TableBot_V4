# TableBot_V4 - Autonomous Table Cleaning Robot

TableBot_V4 is an autonomous, edge-detecting, reactive robot designed to autonomously clean a specified tabletop area without falling off the edge. It features precise continuous motor feedback with quadrature encoders, heading preservation via an MPU6050, and robust obstacle and edge detection through a hybrid IR/Ultrasonic sensor array.

## Features

- **Reactive Autonomous Cleaning**: Discovers the boundaries of the table and systematically bounces inward using precise encoder reversing and gyro turning mechanisms.
- **PID Straight-line Control**: Utilizes the MPU6050 yaw readings coupled with a PID loop to guarantee the robot drives perfectly straight, compensating for uneven motor torque.
- **D-Pad Web Control**: Hosts an embedded web interface, allowing direct RC control via a mobile device connected to its local access point.
- **Real-Time Telemetry**: Broadcasts live data (Distance, Yaw angle, Motor Ticks, Obstacle Proximity, and Edge Hazards) at a fast 2Hz refresh rate for UI visualization.

## Hardware Stack

- ESP32 Development Board (DOIT)
- 2x GM25-370CA DC Gear Motors + Quadrature Encoders (100 RPM / 34:1 Ratio)
- 1x L298N Dual H-Bridge Motor Driver
- 4x IR Obstacle Avoidance Sensors (Edge Detection)
- 1x HC-SR04 Ultrasonic Sensor (Obstacle Avoidance)
- 1x MPU6050 IMU (Heading Control)

## Documentation Structure

For developers looking to adapt or modify the TableBot, refer to the exhaustive documentation in the `docs/` folder:

- [Hardware Setup & Pinouts](docs/Hardware.md)
- [Firmware Architecture & Modes](docs/Firmware.md)
- [Web Interface & APIs](docs/Software.md)

## Getting Started

### Prerequisites
- [PlatformIO Core](https://platformio.org/) installed via VSCode.
- ESP32 USB Drivers (CP210x or CH340).

### Compiling and Flashing
1. Connect the ESP32.
2. Open the `TableBot_V4` folder in your IDE.
3. Depending on your IR sensor threshold behaviour, you might need to adjust `EDGE_ACTIVE` in `include/config.h`. Defaults to `HIGH`.
4. Run `pio run -t upload`.

### Connect and Control
1. Power cycle the robot to activate the 7.4V battery. Wait ~1 second for the MPU6050 to calibrate.
2. On your mobile device or laptop, connect to the new WiFi network:
   - **Network**: `TableBot`
   - **Password**: `TableBot@2025`
3. Open a browser and navigate to `http://192.168.4.1`.
4. Use the D-Pad to drive manually, or enter table dimensions and click "Start Cleaning" for Reactive Mode.

## Recommended GitHub Folder Structure

```text
TableBot_V4/
├── docs/                      # Extensive project documentation
│   ├── Hardware.md            # Circuit info, power limits, and GPIO maps
│   ├── Firmware.md            # ESP32 application architecture logic
│   └── Software.md            # WebServer JSON API and web-app design
├── include/                   # Shared C++ Headers (if any are separate)
├── src/                       # Source Code & Primary Headers
│   ├── main.cpp               # Setup and Loop hooks
│   ├── config.h               # Global constants (Pins, Speed, Constants)
│   ├── MotorControl.h/cpp     # Motor classes, PID, Encoders
│   ├── Sensors.h/cpp          # Ultrasonic, IR, and MPU wrappers
│   ├── ReactiveCleaner.h/cpp  # Core autonomous behavior state machine
│   └── RobotWebServer.h/cpp   # HTML literals, Networking, Web Endpoints
├── platformio.ini             # PlatformIO configuration (Dependencies, Envs)
└── README.md                  # Detailed top-level repository info
```
