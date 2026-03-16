#include "MotorControl.h"
#include "Sensors.h"
#include "config.h"

// ── Encoder ISR counters ──
// Declared volatile + extern in MotorControl.h for telemetry access
volatile long _encTicksL = 0;
volatile long _encTicksR = 0;

void IRAM_ATTR isr_enc_l() {
  // Direction: B HIGH = forward, B LOW = backward
  if (digitalRead(ENC_L_B) == HIGH) _encTicksL++;
  else                               _encTicksL--;
}

void IRAM_ATTR isr_enc_r() {
  if (digitalRead(ENC_R_B) == HIGH) _encTicksR++;
  else                               _encTicksR--;
}

void MotorControl::begin() {
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);

  // GPIO16/17 support INPUT_PULLUP
  // GPIO34/35 are input-only — no internal pull-up available
  // GM25-370CA encoder board provides its own pull-ups ✅
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT);
  pinMode(ENC_R_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isr_enc_l, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isr_enc_r, RISING);

  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_L_EN, PWM_CH_L);
  ledcAttachPin(MOTOR_R_EN, PWM_CH_R);

  stop();
}

// ── Distance helpers ──

void MotorControl::resetSnapshot() {
  _snapTicksL = _encTicksL;
  _snapTicksR = _encTicksR;
}

float MotorControl::getSnapshotDistance() {
  long dL = abs((long)(_encTicksL - _snapTicksL));
  long dR = abs((long)(_encTicksR - _snapTicksR));
  return ((dL + dR) / 2.0f) * CM_PER_TICK;
}

// Legacy helper — kept for web telemetry
void MotorControl::resetDistance() {
  _startTicksL = _encTicksL;
  _startTicksR = _encTicksR;
}

float MotorControl::getDistanceCm() {
  long dL = abs((long)(_encTicksL - _startTicksL));
  long dR = abs((long)(_encTicksR - _startTicksR));
  return ((dL + dR) / 2.0f) * CM_PER_TICK;
}

// ── Raw motor output ──
void MotorControl::setMotors(int leftPWM, int rightPWM) {
  if (leftPWM >= 0) {
    digitalWrite(MOTOR_L_IN1, HIGH);
    digitalWrite(MOTOR_L_IN2, LOW);
  } else {
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, HIGH);
    leftPWM = -leftPWM;
  }
  ledcWrite(PWM_CH_L, constrain(leftPWM, 0, 255));

  if (rightPWM >= 0) {
    digitalWrite(MOTOR_R_IN1, HIGH);
    digitalWrite(MOTOR_R_IN2, LOW);
  } else {
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, HIGH);
    rightPWM = -rightPWM;
  }
  ledcWrite(PWM_CH_R, constrain(rightPWM, 0, 255));
}

// ── Forward with Dual PID (Encoder velocity + Gyro heading) ──
void MotorControl::forward(Sensors &sensors, int speed) {
  int s = (speed < 0) ? BASE_SPEED : speed;
  
  if (!_headingLocked) {
    _targetYaw     = sensors.getYaw();
    _integral      = 0;
    _lastError     = 0;
    _encIntegral   = 0;
    _encLastError  = 0;
    _startTicksL   = _encTicksL;
    _startTicksR   = _encTicksR;
    _headingLocked = true;
  }

  // 1. Encoder Velocity Match (Keeps base wheel speeds identical)
  long diffL = abs((long)(_encTicksL - _startTicksL));
  long diffR = abs((long)(_encTicksR - _startTicksR));
  float encError = diffL - diffR; // Positive if Left is faster

  _encIntegral += encError;
  _encIntegral = constrain(_encIntegral, -50.0f, 50.0f);
  float encDeriv = encError - _encLastError;
  _encLastError = encError;

  // Simple PI for encoders (adjust these constants if needed)
  float encCorr = (0.5f * encError) + (0.01f * _encIntegral);
  encCorr = constrain(encCorr, -20.0f, 20.0f);

  // Apply encoder correction to base speed
  int baseL = constrain(s - (int)encCorr, 0, MAX_SPEED);
  int baseR = constrain(s + (int)encCorr, 0, MAX_SPEED);

  // 2. Gyro Heading Lock (Fixes physical drift from slip/surface)
  float error = _targetYaw - sensors.getYaw();
  _integral += error;
  _integral = constrain(_integral, -100.0f, 100.0f);
  float derivative = error - _lastError;
  _lastError = error;

  float gyroCorr = HEADING_KP * error + HEADING_KI * _integral + HEADING_KD * derivative;
  gyroCorr = constrain(gyroCorr, -HEADING_MAX_CORR, HEADING_MAX_CORR);

  // 3. Combine and set motors
  setMotors(constrain(baseL - (int)gyroCorr, 0, MAX_SPEED),
            constrain(baseR + (int)gyroCorr, 0, MAX_SPEED));
}

// ── Backward with Dual PID ──
void MotorControl::backward(Sensors &sensors, int speed) {
  int s = (speed < 0) ? BASE_SPEED : speed;

  if (!_headingLocked) {
    _targetYaw     = sensors.getYaw();
    _integral      = 0;
    _lastError     = 0;
    _encIntegral   = 0;
    _encLastError  = 0;
    _startTicksL   = _encTicksL;
    _startTicksR   = _encTicksR;
    _headingLocked = true;
  }

  // 1. Encoder Velocity Match
  long diffL = abs((long)(_encTicksL - _startTicksL));
  long diffR = abs((long)(_encTicksR - _startTicksR));
  float encError = diffL - diffR; // Positive if Left is faster

  _encIntegral += encError;
  _encIntegral = constrain(_encIntegral, -50.0f, 50.0f);
  float encDeriv = encError - _encLastError;
  _encLastError = encError;

  float encCorr = (0.5f * encError) + (0.01f * _encIntegral);
  encCorr = constrain(encCorr, -20.0f, 20.0f);

  int baseL = constrain(s - (int)encCorr, 0, MAX_SPEED);
  int baseR = constrain(s + (int)encCorr, 0, MAX_SPEED);

  // 2. Gyro Heading Lock
  float error = _targetYaw - sensors.getYaw();
  _integral += error;
  _integral = constrain(_integral, -100.0f, 100.0f);
  float derivative = error - _lastError;
  _lastError = error;

  float gyroCorr = HEADING_KP * error + HEADING_KI * _integral + HEADING_KD * derivative;
  gyroCorr = constrain(gyroCorr, -HEADING_MAX_CORR, HEADING_MAX_CORR);

  // 3. Combine and set motors reversed
  setMotors(constrain(-(baseL + (int)gyroCorr), -MAX_SPEED, 0),
            constrain(-(baseR - (int)gyroCorr), -MAX_SPEED, 0));
}

// ── Turns ──
void MotorControl::turnLeft(int speed) {
  _headingLocked = false;
  _integral      = 0;
  int s = (speed < 0) ? TURN_SPEED : speed;
  setMotors(-s, s);
}

void MotorControl::turnRight(int speed) {
  _headingLocked = false;
  _integral      = 0;
  int s = (speed < 0) ? TURN_SPEED : speed;
  setMotors(s, -s);
}

void MotorControl::stop() {
  _headingLocked = false;
  _integral      = 0;
  _lastError     = 0;
  setMotors(0, 0); // Coast to stop
}

// ── Emergency Active Brake (Short Circuit Motors) ──
void MotorControl::brake() {
  _headingLocked = false;
  _integral      = 0;
  _lastError     = 0;
  
  // To hard-brake with L298N, set both IN1 and IN2 to HIGH 
  // and PWM to Max. This shorts the motor terminals.
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, HIGH);
  ledcWrite(PWM_CH_L, 255);
  
  digitalWrite(MOTOR_R_IN1, HIGH);
  digitalWrite(MOTOR_R_IN2, HIGH);
  ledcWrite(PWM_CH_R, 255);
}

void MotorControl::resetHeadingLock() {
  _headingLocked = false;
  _integral      = 0;
  _lastError     = 0;
}
