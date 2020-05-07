#include "api.h"

#include <bits/stdc++.h>

#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"

namespace controllerbuttons {

void empty_function() {}
pros::Task empty_task(empty_function);

std::vector<MacroGroup *> macro_group_vector;
// Stores what buttons should run which functions
// Is writen to in ::setCalback functions
std::vector<ButtonStruct> button_callbacks;

bool is_task_running(int task_state) {
  return (task_state == pros::E_TASK_STATE_RUNNING || task_state == pros::E_TASK_STATE_READY || task_state == pros::E_TASK_STATE_BLOCKED);
}

void runButtons() {
  // for (auto &macro_group : macro_group_vector) {
  for(int i = 0; i < macro_group_vector.size(); i++) {
    // macro_group->is_running = is_task_running(macro_group->last_run_task->get_state());
    macro_group_vector[i]->is_running = is_task_running(macro_group_vector[i]->last_run_task->get_state());
    printf("group: %d, state: %d\r\n", i, macro_group_vector[i]->last_run_task->get_state());
  }

  // Cycle through all button callbacks
  for (auto &button_callback : button_callbacks) {
    bool is_pressing = button_callback.controller->get_digital(button_callback.button);
    bool was_pressed  = (is_pressing &&
                         !button_callback.was_triggered);
    bool was_released = (!is_pressing &&
                         button_callback.was_triggered);
    bool is_running = false;
    for (auto &macro_group : button_callback.macro_groups) {
      is_running = is_running || macro_group->is_running;
    }

    // If the button has been pressed and the task isn't running
    if (was_pressed && !is_running) {
      // Set the function to not run when the button is held
      button_callback.was_triggered = true;
      if (button_callback.trigger_on_release) continue;
    } else if (was_released) {
      button_callback.was_triggered = false;
      if (!button_callback.trigger_on_release) continue;
    } else {
      continue;
    }

    // Run the function in a separate task
    button_callback.button_task = button_callback.function;
    for (auto &macro_group : button_callback.macro_groups) {
      macro_group->is_running = true;
      macro_group->last_run_task = &button_callback.button_task;
    }
  }
}

void interruptMacroGroup(std::vector<MacroGroup *> macro_groups) {
  for (auto &macro_group : macro_groups) {
    macro_group->last_run_task->remove();
  }
}
} // namespace controllerbuttons
