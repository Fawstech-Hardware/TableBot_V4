#include "MotorControl.h"
#include "ReactiveCleaner.h"
#include "RobotWebServer.h"
#include "Sensors.h"
#include "config.h"
#include <Arduino.h>

MotorControl   motors;
Sensors        sensors;
ReactiveCleaner cleaner;

enum RobotMode { MODE_IDLE, MODE_MANUAL, MODE_AUTO };
RobotMode robotMode = MODE_IDLE;

RobotWeb::Command lastManualCmd = RobotWeb::CMD_NONE;

const char *modeName() {
  if (robotMode == MODE_AUTO && cleaner.isRunning())
    return cleaner.getStateName();
  switch (robotMode) {
    case MODE_MANUAL: return "MANUAL";
    case MODE_AUTO:   return "AUTO";
    default:          return "IDLE";
  }
}

void processWebCommand() {
  RobotWeb::Command cmd = RobotWeb::getCommand();
  if (cmd == RobotWeb::CMD_NONE) return;
  RobotWeb::clearCommand();

  Serial.printf("[CMD] %d | EdgeF L=%d R=%d | EdgeR L=%d R=%d | Obs=%.1f\n",
                cmd,
                sensors.edgeLeft(), sensors.edgeRight(),
                sensors.edgeRearLeft(), sensors.edgeRearRight(),
                sensors.obstacleCm());

  switch (cmd) {

  case RobotWeb::CMD_FWD:
    robotMode     = MODE_MANUAL;
    lastManualCmd = RobotWeb::CMD_FWD;
    motors.resetHeadingLock();
    sensors.resetYaw();
    break;

  case RobotWeb::CMD_BWD:
    robotMode     = MODE_MANUAL;
    lastManualCmd = RobotWeb::CMD_BWD;
    motors.resetHeadingLock();
    sensors.resetYaw();
    break;

  case RobotWeb::CMD_LEFT:
    robotMode     = MODE_MANUAL;
    lastManualCmd = RobotWeb::CMD_LEFT;
    motors.turnLeft();
    break;

  case RobotWeb::CMD_RIGHT:
    robotMode     = MODE_MANUAL;
    lastManualCmd = RobotWeb::CMD_RIGHT;
    motors.turnRight();
    break;

  case RobotWeb::CMD_STOP:
    robotMode     = MODE_IDLE;
    lastManualCmd = RobotWeb::CMD_NONE;
    motors.stop();
    cleaner.stop();
    break;

  case RobotWeb::CMD_AUTO_START:
    robotMode     = MODE_AUTO;
    lastManualCmd = RobotWeb::CMD_NONE;
    motors.stop();
    cleaner.start();
    break;

  case RobotWeb::CMD_AUTO_STOP:
    cleaner.stop();
    motors.stop();
    robotMode     = MODE_IDLE;
    lastManualCmd = RobotWeb::CMD_NONE;
    break;

  default: break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== AutoCleanBot V4 Starting ===");
  motors.begin();
  sensors.begin();   // gyro calibrates ~1s — keep still!
  RobotWeb::begin();
  Serial.println("[OK] Ready. Connect to WiFi: " WIFI_SSID);
  Serial.println("[TIP] If edge detection is backwards, flip EDGE_ACTIVE in config.h");
}

void loop() {
  // 1. Web server
  RobotWeb::handle();

  // 2. All sensors
  sensors.update();

  // 3. Incoming commands
  processWebCommand();

  // 4. Manual mode
  if (robotMode == MODE_MANUAL) {
    switch (lastManualCmd) {

    case RobotWeb::CMD_FWD:
      // Safety: only stop forward at front edge or obstacle
      if (sensors.anyEdge()) {
        motors.brake();
        lastManualCmd = RobotWeb::CMD_NONE;
        robotMode     = MODE_IDLE;
        Serial.println("[EDGE] Front edge — stopped forward.");
      } else if (sensors.obstacleDetected()) {
        motors.stop();
        lastManualCmd = RobotWeb::CMD_NONE;
        robotMode     = MODE_IDLE;
        Serial.printf("[OBS] Obstacle %.1fcm — stopped.\n", sensors.obstacleCm());
      } else {
        motors.forward(sensors); // PID every loop
      }
      break;

    case RobotWeb::CMD_BWD:
      // Safety: only stop backward at rear edge
      if (sensors.anyRearEdge()) {
        motors.brake();
        lastManualCmd = RobotWeb::CMD_NONE;
        robotMode     = MODE_IDLE;
        Serial.println("[EDGE] Rear edge — stopped backward.");
      } else {
        motors.backward(sensors); // PID every loop
      }
      break;

    case RobotWeb::CMD_LEFT:
      // FIX: re-apply turn every loop — not just once on button press
      // No edge/obstacle check during turns — user controls direction
      motors.turnLeft();
      break;

    case RobotWeb::CMD_RIGHT:
      // FIX: re-apply turn every loop
      motors.turnRight();
      break;

    default:
      break;
    }
  }

  // 5. Auto mode
  if (robotMode == MODE_AUTO && cleaner.isRunning()) {
    cleaner.update(motors, sensors);
    if (!cleaner.isRunning()) {
      motors.stop();
      robotMode = MODE_IDLE;
    }
  }

  // 6. Telemetry
  extern volatile long _encTicksL;
  extern volatile long _encTicksR;
  RobotWeb::setTelemetry(
    sensors.obstacleCm(),
    sensors.edgeLeft(), sensors.edgeRight(),
    sensors.edgeRearLeft(), sensors.edgeRearRight(),
    sensors.getYaw(),
    motors.getDistanceCm(), // distCm
    _encTicksL, _encTicksR, // ticksL, ticksR
    0, 0,                   // currentPass, totalPasses 
    modeName());

  delay(10);
}
