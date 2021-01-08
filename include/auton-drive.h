#ifndef AUTON_DRIVE_H
#define AUTON_DRIVE_H

#include "controller-buttons.h"

#define WAIT_UNTIL(condition) \
while (!(condition)) {        \
pros::delay(5);               \
}

namespace autondrive {

class Target {
 public:
  Target(QLength x, QLength y, QAngle theta);

  static double forward;
  static double strafe;
  static double turn;

  QLength x = 0_in;
  QLength y = 0_in;
  QAngle theta = 0_deg;
  OdomState starting_state;
  int millis_at_start;

  bool is_new = true;
  void init_if_new();
};

namespace drivetoposition {
void addPositionTarget(QLength x, QLength y, QAngle theta, QLength offset = 0_in);
extern bool targetPositionEnabled;
extern bool final_target_reached;
}

extern controllerbuttons::Macro main_auton;
extern controllerbuttons::Macro shawnton;

void motor_task();
void set_callbacks();
} // namespace autondrive

#endif // AUTON_CONTROLLER_H