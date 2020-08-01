#ifndef CONTROLLER_BUTTONS_H
#define CONTROLLER_BUTTONS_H

// #define BTN_A     pros::E_CONTROLLER_DIGITAL_A
// #define BTN_Y     pros::E_CONTROLLER_DIGITAL_Y
// #define BTN_X     pros::E_CONTROLLER_DIGITAL_X
// #define BTN_RIGHT pros::E_CONTROLLER_DIGITAL_RIGHT
// #define BTN_LEFT  pros::E_CONTROLLER_DIGITAL_LEFT
// #define BTN_LEFT  pros::E_CONTROLLER_DIGITAL_LEFT
// #define BTN_UP    pros::E_CONTROLLER_DIGITAL_UP
// #define BTN_B     pros::E_CONTROLLER_DIGITAL_B
// #define BTN_DOWN  pros::E_CONTROLLER_DIGITAL_DOWN

#include "api.h"
#include <bits/stdc++.h>

namespace controllerbuttons {
extern bool unnasigned_group;

struct MacroGroup {
  bool * is_running_ptr = &unnasigned_group;
  pros::task_t * group_task_t;
};

struct ButtonStruct {
  pros::Controller * controller;
  pros::controller_digital_e_t button;
  std::function<void()> function;
  bool trigger_on_release = false;
  std::vector<MacroGroup *> macro_groups;
  bool run_as_task = false;

  bool was_triggered = true;
  bool is_running;
  pros::task_t button_task_t;
};



/**
 * Checks through each struct in the vector one by one,
 * and runs starts the function running on a seperate task if:
 *   1. The associated button is being pressed,
 *   2. There is not currently a function running in the same group,
 *   3. Last time around the loop, either the button wasn't pressed,
 *      or the group was running.
 * If the function is set to trigger_on_release, then it will run
 * when the button is released, instead of pressed.
 *
 * Subgroups are only for external reference, and do not effect what the buttons do.
 *
 * Should be run in a loop.
 */
void interrupt_macro_group(MacroGroup * group);

void run_buttons();

void set_callbacks();

// Stores what buttons should run which functions
// Is writen to in ::setCallback functions
extern std::vector<ButtonStruct> button_callbacks;

// extern std::vector<std::vector<ButtonStruct>> button_handler;

} // namespace controllerbuttons

#endif // CONTROLLER_BUTTONS_H
