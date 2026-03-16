#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

class Sensors {
public:
  void begin();
  void update(); // poll sensors each loop

  // Edge detection (front)
  bool edgeLeft() const { return _edgeL; }
  bool edgeRight() const { return _edgeR; }
  bool anyEdge() const { return _edgeL || _edgeR; }

  // Edge detection (rear)
  bool edgeRearLeft() const { return _edgeRearL; }
  bool edgeRearRight() const { return _edgeRearR; }
  bool anyRearEdge() const { return _edgeRearL || _edgeRearR; }

  // Obstacle detection
  float obstacleCm() const { return _distCm; }
  bool obstacleDetected() const;

  // MPU6050 heading
  float getYaw() const { return _yaw; }    // degrees, cumulative
  void resetYaw();                         // zero the yaw counter
  bool mpuOk() const { return _mpuReady; } // false if init failed

private:
  float readUltrasonic();
  void initMPU();
  void updateMPU();

  // IR + ultrasonic
  bool _edgeL = false;
  bool _edgeR = false;
  bool _edgeRearL = false;
  bool _edgeRearR = false;
  float _distCm = 999.0f;
  unsigned long _lastUS = 0;
  static const unsigned long US_INTERVAL = 100;

  // MPU6050
  bool _mpuReady = false;
  float _yaw = 0;         // integrated yaw in degrees
  float _gyroZoffset = 0; // calibration offset
  unsigned long _lastMPU = 0;
};

#endif // SENSORS_H
