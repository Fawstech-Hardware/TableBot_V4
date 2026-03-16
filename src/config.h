#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
//  AutoCleanBot – Master Configuration
//  Board  : ESP32 DevKit V1 (DOIT)
//  Motors : GM25-370CA with quadrature encoders
//  Wheel  : 65mm diameter | Circumference: 20.42cm
//  Track  : 124mm centre-to-centre
//  Table  : 60cm × 45cm
// ============================================================

// -------------------- Motor Driver (L298N) ------------------
#define MOTOR_L_EN   32  // ENA – Left PWM  (GPIO5 is strapping pin — avoid!)
#define MOTOR_L_IN1  19
#define MOTOR_L_IN2  23
#define MOTOR_R_EN   33  // ENB – Right PWM
#define MOTOR_R_IN1  25
#define MOTOR_R_IN2  26

#define PWM_FREQ  1000
#define PWM_RES   8       // 8-bit → 0–255
#define PWM_CH_L  0
#define PWM_CH_R  1

// -------------------- Encoders (GM25-370CA) -----------------
// Left encoder
#define ENC_L_A  16   // Phase A — interrupt (GPIO16 has internal pull-up)
#define ENC_L_B  17   // Phase B — direction (GPIO17 has internal pull-up)
// Right encoder
#define ENC_R_A  34   // Phase A — interrupt (GPIO34 INPUT-ONLY, no pull-up!)
#define ENC_R_B  35   // Phase B — direction (GPIO35 INPUT-ONLY, no pull-up!)
// ⚠️  GPIO34/35 have NO internal pull-up. The GM25-370CA encoder board
//    has its own 10kΩ pull-ups on the signal lines — this is fine.

// GM25-370CA encoder calculation:
//   Motor shaft PPR : 11 (rising edges on one channel)
//   Gear ratio      : 34:1  (100 RPM model)
//   Ticks/wheel rev : 11 × 34 = 374
//   Wheel circum    : π × 6.5cm = 20.42cm
//   CM_PER_TICK     : 20.42 / 374 = 0.0546 cm/tick
#define CM_PER_TICK  0.0546f

// -------------------- IR Edge Sensors -----------------------
#define IR_LEFT        36   // VP – front left  (input-only)
#define IR_RIGHT       39   // VN – front right (input-only)
#define IR_REAR_LEFT   14   // rear left
#define IR_REAR_RIGHT  27   // rear right

// ⚠️  If edge detection is inverted (triggers on surface, not on edge):
//    Change HIGH → LOW below and re-flash.
//    Run diagnostic build to check raw values before changing.
// HIGH usually means no reflection = edge (for standard IR modules)
#define EDGE_ACTIVE  HIGH

// -------------------- Ultrasonic (HC-SR04) ------------------
#define US_TRIG  4
#define US_ECHO  18
#define OBSTACLE_CM  12

// -------------------- MPU6050 IMU (I2C) ---------------------
#define MPU_SDA  21
#define MPU_SCL  22
#define MPU_ADDR  0x68
#define GYRO_SCALE  131.0f

// -------------------- Robot Dimensions ----------------------
#define ROBOT_WIDTH_CM   20.0f
#define TRACK_WIDTH_CM   12.4f
#define WHEEL_DIAM_CM    6.5f
#define WHEEL_CIRCUM_CM  20.42f

// -------------------- Motor Speeds --------------------------
// GM25-370CA is a metal gear motor — more torque than BO motors.
// At 7.4V battery with L298N ~1.3V drop = ~6.1V effective.
#define BASE_SPEED  190   // forward / backward (increased to prevent stall)
#define TURN_SPEED  170   // in-place turns
#define MAX_SPEED   230

// -------------------- Gyro Heading PID ----------------------
#define HEADING_KP        0.6f
#define HEADING_KI        0.005f
#define HEADING_KD        1.2f
#define HEADING_MAX_CORR  25.0f
#define GYRO_DEADBAND     1.0f

// -------------------- Reactive Cleaner ----------------------
// How far to reverse after hitting an edge (encoder-based, precise)
#define REVERSE_CM        10.0f   // reverse 10cm away from edge

// Fallback reverse timer if encoders read 0 (wiring fault)
// At BASE_SPEED=170 (~20cm/s): 10cm / 20cm/s = 500ms
#define REVERSE_MS_FALLBACK  600

// Turn angle: ~120° gives good random coverage across table
// Gyro primary. Fallback timer:
//   Arc = (120/360) × π × 12.4cm = 12.98cm
//   At TURN_SPEED=110 (~12.7cm/s) = 1022ms → 1100ms
#define TURN_DEGREES      115.0f
#define TURN_MS_FALLBACK  1100

// Rear sensor debounce during reverse
#define REAR_DEBOUNCE_MS  150

// -------------------- WiFi ----------------------------------
#define WIFI_SSID  "TableBot"
#define WIFI_PASS  "TableBot@2025"

#endif // CONFIG_H
