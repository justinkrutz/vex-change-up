#ifndef CONTROLLER_BUTTONS_H
#define CONTROLLER_BUTTONS_H

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
  // Button to be checked
  pros::controller_digital_e_t button;
  // Should the functon be started when the button us pressed or released
  bool trigger_on_release;
  std::vector<MacroGroup *> macro_groups;
  void (*function)();

  bool was_triggered;
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
void interruptMacroGroup(MacroGroup * group);

void run_buttons();

extern std::vector<MacroGroup *> macro_group_vector;

// Stores what buttons should run which functions
// Is writen to in ::setCallback functions
extern std::vector<ButtonStruct> button_callbacks;

} // namespace controllerbuttons

#endif // CONTROLLER_BUTTONS_H
