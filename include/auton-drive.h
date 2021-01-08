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
extern bool target_position_enabled;
extern bool final_target_reached;
}

extern controllerbuttons::Macro left_home_row;
extern controllerbuttons::Macro shawnton_right;

void motor_task();
void set_callbacks();
} // namespace autondrive

#endif // AUTON_CONTROLLER_H