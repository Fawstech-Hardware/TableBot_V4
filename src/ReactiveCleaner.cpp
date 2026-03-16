#include "ReactiveCleaner.h"
#include "config.h"

const char *ReactiveCleaner::getStateName() const {
  switch (_state) {
    case IDLE:      return "IDLE";
    case FORWARD:   return "CLEANING";
    case REVERSING: return "REVERSE";
    case TURNING:   return "TURNING";
    default:        return "UNKNOWN";
  }
}

void ReactiveCleaner::start() {
  _state    = FORWARD;
  _turnDir  = 1;
  _timer    = 0;
  Serial.println("[REACTIVE] Started — edge bounce cleaning");
  Serial.println("[REACTIVE] Front edge → reverse 10cm → turn 115° → repeat");
}

void ReactiveCleaner::stop() {
  _state = IDLE;
}

void ReactiveCleaner::update(MotorControl &motors, Sensors &sensors) {
  if (_state == IDLE) return;

  switch (_state) {

  // ── Drive forward, gyro PID holds straight line ──
  case FORWARD: {
    if (sensors.anyEdge()) {
      motors.brake();
      Serial.printf("[REACTIVE] Front edge! L=%d R=%d  Yaw=%.1f\n",
                    sensors.edgeLeft(), sensors.edgeRight(),
                    sensors.getYaw());
      motors.resetSnapshot();
      _reverseStart = millis();
      _state = REVERSING;
      return;
    }
    if (sensors.obstacleDetected()) {
      motors.stop();
      Serial.printf("[REACTIVE] Obstacle at %.1fcm\n", sensors.obstacleCm());
      motors.resetSnapshot();
      _reverseStart = millis();
      _state = REVERSING;
      return;
    }
    motors.forward(sensors);
    break;
  }

  // ── Reverse exact distance using encoders ──
  // Fallback to timer if encoder ticks not moving (wiring fault)
  case REVERSING: {
    float distMoved = motors.getSnapshotDistance();
    // IMMEDIATE STOP: Do not wait for debounce when reversing towards an edge
    bool rearEdge   = sensors.anyRearEdge();
    bool encDone    = (distMoved >= REVERSE_CM);
    bool timerDone  = (millis() - _reverseStart >= REVERSE_MS_FALLBACK);

    if (rearEdge) {
      Serial.println("[REACTIVE] Rear edge during reverse — turning now");
      motors.brake();
    } else if (encDone) {
      Serial.printf("[REACTIVE] Reversed %.1fcm — turning\n", distMoved);
      motors.stop();
    } else if (timerDone) {
      Serial.println("[REACTIVE] Reverse fallback timer — turning");
      Serial.println("  [!] Check encoder wiring if this triggers every time!");
      motors.stop();
    } else {
      motors.backward(sensors);
      return;
    }

    // Brief physical settle before turn
    delay(60);
    sensors.resetYaw();

    if (_turnDir > 0) motors.turnLeft();
    else              motors.turnRight();

    _state = TURNING;
    _timer = millis();
    Serial.printf("[REACTIVE] Turning %s\n", _turnDir > 0 ? "LEFT" : "RIGHT");
    break;
  }

  // ── Turn ~115°, gyro primary / timer fallback ──
  case TURNING: {
    float yaw        = fabsf(sensors.getYaw());
    bool gyroReached = sensors.mpuOk() && (yaw >= TURN_DEGREES);
    bool timeReached = (millis() - _timer) >= TURN_MS_FALLBACK;

    if (gyroReached || timeReached) {
      motors.stop();
      _turnDir *= -1; // next turn goes opposite direction
      _state    = FORWARD;
      Serial.printf("[REACTIVE] Turn done (%.1f°). Forward.\n", yaw);
    }
    break;
  }

  default: break;
  }
}
