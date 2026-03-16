# TableBot_V4 Hardware Documentation

This document describes the hardware components, circuit design, and pin configurations for the TableBot_V4 edge-cleaning robot.

## Core Components

- **Microcontroller**: ESP32 DevKit V1 (DOIT)
  - Handles web server hosting, PID motor control, and sensor data processing.
- **Motor Driver**: L298N
  - Dual H-Bridge motor driver used to power and control the direction of the two DC motors.
- **Motors**: 2x GM25-370CA DC Gear Motors (100 RPM)
  - Metal gear motors providing higher torque than standard BO motors.
  - Built-in quadrature encoders for precise distance measurement and speed control (374 ticks per wheel revolution).
- **Wheels**: 65mm diameter rubber wheels.
  - Track Width (centre-to-centre): 124mm
  - Circumference: 20.42cm
- **Edge Detection**: 4x IR Obstacle Avoidance Sensors
  - Mounted pointing downwards to detect table edges.
  - 2 Front (Left/Right) and 2 Rear (Left/Right).
- **Obstacle Detection**: HC-SR04 Ultrasonic Sensor
  - Mounted facing forward to detect objects in the robot's path.
- **IMU**: MPU6050 (Accelerometer + Gyroscope)
  - Used for maintaining a straight heading (yaw correction) and executing precise angle turns (e.g., 115° turns during auto-cleaning).

## Pin Configuration (ESP32)

### Motors (L298N)
| ESP32 Pin | L298N Pin | Function |
| :--- | :--- | :--- |
| GPIO 32 | ENA | Left Motor PWM |
| GPIO 19 | IN1 | Left Motor Direction 1 |
| GPIO 23 | IN2 | Left Motor Direction 2 |
| GPIO 33 | ENB | Right Motor PWM |
| GPIO 25 | IN1 | Right Motor Direction 1 |
| GPIO 26 | IN2 | Right Motor Direction 2 |

### Encoders (GM25-370CA)
| ESP32 Pin | Encoder | Function | Notes |
| :--- | :--- | :--- | :--- |
| GPIO 16 | Left A | Interrupt (Phase A) | Internal Pull-up |
| GPIO 17 | Left B | Direction (Phase B) | Internal Pull-up |
| GPIO 34 | Right A | Interrupt (Phase A) | Input only, uses encoder pull-up |
| GPIO 35 | Right B | Direction (Phase B) | Input only, uses encoder pull-up |

### Sensors
| Component | ESP32 Pin | Function |
| :--- | :--- | :--- |
| IR Front Left | GPIO 36 (VP) | Edge Detection |
| IR Front Right | GPIO 39 (VN) | Edge Detection |
| IR Rear Left | GPIO 14 | Edge Detection |
| IR Rear Right | GPIO 27 | Edge Detection |
| HC-SR04 Trig | GPIO 4 | Ultrasonic Trigger |
| HC-SR04 Echo | GPIO 18 | Ultrasonic Echo |
| MPU6050 SDA | GPIO 21 | I2C Data |
| MPU6050 SCL | GPIO 22 | I2C Clock |

## Power Considerations
- The robot is powered by a 7.4V battery pack (e.g., 2x 18650 Li-ion cells).
- The L298N motor driver drops approx 1.3V, providing ~6.1V effective voltage to the motors.
- The ESP32 is powered via the 5V output of the L298N or a dedicated step-down converter (LM2596) if cleaner power is required.
