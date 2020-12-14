#ifndef AUTON_CONTROLLER_H
#define AUTON_CONTROLLER_H

#include "controller-buttons.h"


namespace autoncontroller {

extern controllerbuttons::Macro main_auton;
extern controllerbuttons::Macro shawnton;

void motor_task();
void set_callbacks();
} // namespace autoncontroller

#endif // AUTON_CONTROLLER_H