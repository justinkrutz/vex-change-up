#include "api.h"

#include <bits/stdc++.h>

#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"

namespace controllerbuttons {

bool unnasigned_group;

pros::Mutex mutex;

std::vector<MacroGroup *> macro_group_vector;

// Stores what buttons should run which functions
// Is writen to in ::setCalback functions
std::vector<ButtonStruct> button_callbacks;

void task_start_wrapper(void * void_ptr) {
  ButtonStruct * struct_ptr = (ButtonStruct*) void_ptr;
  for(auto group : struct_ptr->macro_groups) {
    group->is_running_ptr = &struct_ptr->is_running;
    group->group_task_t = &struct_ptr->button_task_t;
  }

  struct_ptr->is_running = true;
  struct_ptr->function();
  struct_ptr->is_running = false;
  // pros::delay(20); // Wait to exit to prevent rare crash
  mutex.take(TIMEOUT_MAX);
  mutex.give();
  printf("ended\r\n");
}

void interruptMacroGroup(MacroGroup * group) {
  mutex.take(TIMEOUT_MAX);
  // pros::c::task_notify(*group->group_task_t);
  printf("is_running_ptr = %d\r\n", *group->is_running_ptr);
  if (*group->is_running_ptr) {

    pros::delay(500); // REMOVE THIS!!!!

    pros::c::task_delete(*group->group_task_t);
    printf("DELETED!\r\n");
    *group->is_running_ptr = false;
  }
  mutex.give();
}

void run_buttons() {
  // Cycle through all button callbacks
  for (auto &button_callback : button_callbacks) {
    bool is_pressing = button_callback.controller->get_digital(button_callback.button);
    bool was_pressed  = (is_pressing &&
                         !button_callback.was_triggered);
    bool was_released = (!is_pressing &&
                         button_callback.was_triggered);
    bool is_running = false;
    for (auto &macro_group : button_callback.macro_groups) {
      is_running = is_running || *macro_group->is_running_ptr;
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
    button_callback.button_task_t =
        pros::c::task_create(task_start_wrapper, (void *)&button_callback,
            TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "run_buttons");
  }
}
} // namespace controllerbuttons
