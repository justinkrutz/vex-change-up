#ifndef AUTON_DRIVE_H
#define AUTON_DRIVE_H

#include "controller-buttons.h"

#define WAIT_UNTIL(condition) \
while (!(condition)) {        \
  pros::delay(5);             \
}

#define WAIT_UNTIL_T(condition, timeout) \
int wait_until_timeout_start = pros::millis(); \
while (!(condition) && pros::millis() - wait_until_timeout_start < timeout) { \
  pros::delay(5); \
}

namespace autondrive {
extern controllerbuttons::MacroGroup auton_group;
extern controllerbuttons::MacroGroup drive_group;

OdomState robot_to_tracking_coords (OdomState robot_coords);

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
void add_position_target(QLength x, QLength y, QAngle theta, QLength offset = 0_in);
extern bool target_position_enabled;
extern bool final_target_reached;
}

extern controllerbuttons::Macro goal_center;

void motor_task();
void set_callbacks();
} // namespace autondrive


namespace autonroutines {

extern controllerbuttons::Macro none;
extern controllerbuttons::Macro home_row_three;
extern controllerbuttons::Macro home_row_two;
extern controllerbuttons::Macro left_shawnton;
extern controllerbuttons::Macro right_shawnton;
extern controllerbuttons::Macro test;
extern controllerbuttons::Macro skills;
extern controllerbuttons::Macro shawnton_three;
extern controllerbuttons::Macro shawnton_cycle;

} // namespace autonroutines

#endif // AUTON_CONTROLLER_H