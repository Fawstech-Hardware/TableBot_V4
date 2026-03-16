#include "Sensors.h"
#include "config.h"
#include <Wire.h>

// ── General init ──
void Sensors::begin() {
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_REAR_LEFT, INPUT);
  pinMode(IR_REAR_RIGHT, INPUT);
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);
  digitalWrite(US_TRIG, LOW);

  initMPU();
}

// ── MPU6050 initialisation + calibration ──
void Sensors::initMPU() {
  Wire.begin(MPU_SDA, MPU_SCL);
  Wire.setClock(400000);

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00); // Clear sleep bit
  if (Wire.endTransmission() != 0) {
    Serial.println("[MPU] ERROR: Not found on I2C bus!");
    _mpuReady = false;
    return;
  }

  // Set gyro range to ±250°/s (most sensitive)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Set low-pass filter to ~44 Hz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission();

  _mpuReady = true;
  Serial.println("[MPU] Initialised. Calibrating gyro (hold still)...");

  // Calibrate: average Z-axis offset over 200 samples (~1 sec)
  float sum = 0;
  int samples = 200;
  for (int i = 0; i < samples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2);
    int16_t raw = (Wire.read() << 8) | Wire.read();
    sum += raw;
    delay(5);
  }
  _gyroZoffset = sum / samples;
  _yaw = 0;
  _lastMPU = micros();
  Serial.printf("[MPU] Calibrated. Z-offset = %.1f\n", _gyroZoffset);
}

// ── Read gyro Z and integrate yaw ──
void Sensors::updateMPU() {
  if (!_mpuReady)
    return;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2);

  if (Wire.available() < 2)
    return;

  int16_t raw = (Wire.read() << 8) | Wire.read();
  float gyroZ = (raw - _gyroZoffset) / GYRO_SCALE;

  unsigned long now = micros();
  float dt = (now - _lastMPU) / 1000000.0f;
  _lastMPU = now;

  // FIX 4: Dead-zone at 1.0°/s — compromise between noise rejection
  // and sensitivity. MPU6050 has ~1°/s noise floor after calibration.
  if (abs(gyroZ) > 1.0f) {
    _yaw += gyroZ * dt;
  }
}

void Sensors::resetYaw() { _yaw = 0; }

// ── Main update ──
void Sensors::update() {
  _edgeL = (digitalRead(IR_LEFT) == EDGE_ACTIVE);
  _edgeR = (digitalRead(IR_RIGHT) == EDGE_ACTIVE);
  _edgeRearL = (digitalRead(IR_REAR_LEFT) == EDGE_ACTIVE);
  _edgeRearR = (digitalRead(IR_REAR_RIGHT) == EDGE_ACTIVE);

  unsigned long now = millis();
  if (now - _lastUS >= US_INTERVAL) {
    _distCm = readUltrasonic();
    _lastUS = now;
  }

  updateMPU();
}

bool Sensors::obstacleDetected() const { return _distCm <= OBSTACLE_CM; }

float Sensors::readUltrasonic() {
  digitalWrite(US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG, LOW);

  long duration = pulseIn(US_ECHO, HIGH, 30000);
  if (duration == 0)
    return 999.0f;
  return (duration * 0.0343f) / 2.0f;
}
