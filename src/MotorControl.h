#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

class Sensors;

// Encoder tick counters — updated by ISR, accessible for telemetry
extern volatile long _encTicksL;
extern volatile long _encTicksR;

class MotorControl {
public:
  void begin();

  // Movement with gyro heading correction
  void forward (Sensors &sensors, int speed = -1);
  void backward(Sensors &sensors, int speed = -1);

  // Raw turns (no heading PID)
  void turnLeft (int speed = -1);
  void turnRight(int speed = -1);
  void stop();
  void brake(); // Active braking mechanism
  void resetHeadingLock();
  void setMotors(int leftPWM, int rightPWM);

  // Encoder distance helpers
  void  resetSnapshot();
  float getSnapshotDistance();
  void  resetDistance();                    // legacy reset for telemetry
  float getDistanceCm();                    // legacy helper

private:
  float _integral      = 0;
  float _lastError     = 0;
  float _targetYaw     = 0;
  bool  _headingLocked = false;

  long  _startTicksL   = 0;
  long  _startTicksR   = 0;
  long  _snapTicksL    = 0;
  long  _snapTicksR    = 0;
  
  // Encoder Velocity PID variables
  float _encIntegral   = 0;
  float _encLastError  = 0;
};

#endif
