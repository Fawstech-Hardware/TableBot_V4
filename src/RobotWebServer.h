#ifndef ROBOT_WEB_SERVER_H
#define ROBOT_WEB_SERVER_H

#include <Arduino.h>

namespace RobotWeb {
void begin();
void handle();

enum Command {
  CMD_NONE,
  CMD_FWD,
  CMD_BWD,
  CMD_LEFT,
  CMD_RIGHT,
  CMD_STOP,
  CMD_AUTO_START,
  CMD_AUTO_STOP,
  CMD_AI_STOP,
  CMD_AI_RESUME
};

Command getCommand();
void clearCommand();

struct TableParams {
  float lengthCm;
  float widthCm;
  bool valid;
};
TableParams getTableParams();

// Full telemetry including rear IR and encoders
void setTelemetry(float obstacleCm, bool edgeL, bool edgeR,
                  bool edgeRL, bool edgeRR,
                  float yaw, float distCm,
                  long ticksL, long ticksR,
                  int currentPass, int totalPasses, const char *mode);
} // namespace RobotWeb

#endif // ROBOT_WEB_SERVER_H
