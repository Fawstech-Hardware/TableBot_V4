#ifndef REACTIVE_CLEANER_H
#define REACTIVE_CLEANER_H

#include "MotorControl.h"
#include "Sensors.h"
#include <Arduino.h>

class ReactiveCleaner {
public:
  enum State { IDLE, FORWARD, REVERSING, TURNING };

  void        start();
  void        stop();
  void        update(MotorControl &motors, Sensors &sensors);
  State       getState()     const { return _state; }
  const char *getStateName() const;
  bool        isRunning()    const { return _state != IDLE; }

private:
  State         _state        = IDLE;
  unsigned long _timer        = 0;
  unsigned long _reverseStart = 0;
  int           _turnDir      = 1;
  long          _startTicks   = 0;   // encoder snapshot at reverse start
};

#endif
