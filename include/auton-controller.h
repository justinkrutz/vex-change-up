#ifndef AUTON_CONTROLLER_H
#define AUTON_CONTROLLER_H

#include "controller-buttons.h"

#define WAIT_UNTIL(condition) \
while (!(condition)) {        \
pros::delay(5);               \
}

namespace autoncontroller {

struct Position {
  QLength x = 0_in;
  QLength y = 0_in;
  QAngle theta = 0_deg;
  bool is_new = true;
  OdomState starting_state;
};

namespace drivetoposition {
void addPositionTarget(QLength x, QLength y, QAngle theta, QLength offset = 0_in);
bool targetPositionEnabled = false;
bool final_target_reached = true;
}

extern controllerbuttons::Macro main_auton;
extern controllerbuttons::Macro shawnton;

void motor_task();
void set_callbacks();
} // namespace autoncontroller

#endif // AUTON_CONTROLLER_H