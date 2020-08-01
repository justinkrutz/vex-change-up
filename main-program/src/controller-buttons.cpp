#include "api.h"

#include <bits/stdc++.h>

#include "controller-buttons.h"
#include "robot-config.h"


namespace controllerbuttons {


class ButtonHandler {
  private:
  class ControllerCallback {
    private:
    class ButtonCallback {
      private:
      class ButtonTrigger {
        private:
        // bool run_as_task_;
        
        // pros::task_t button_task_t_;

        // void task_start() {
        //   for(auto group : macro_groups) {
        //     group->is_running_ptr = &is_running;
        //     group->group_task_t = &button_task_t_;
        //   }

        //   is_running = true;
        //   function_();
        //   is_running = false;
        //   // mutex.take(TIMEOUT_MAX);
        //   // mutex.give();
        //   printf("ended\r\n");
        // }
        
        public:
        pros::Controller * controller_;
        pros::controller_digital_e_t button_;

        std::function<void()> function_;
        bool was_triggered_ = false;
        // bool is_running;
        bool is_set = false;
        bool trigger_on_release;
        // std::vector<MacroGroup *> macro_groups;

        ButtonTrigger(pros::Controller * controller, pros::controller_digital_e_t button, bool trigger_on_release_arg) {
          controller_ = controller;
          button_ = button;
          trigger_on_release = trigger_on_release_arg;
        }

        void set(std::function<void()> function) {
          function_ = function;
          is_set = true;
        }

        void clear() {
          is_set = false;
        }

        void run_if_triggered() {
          // printf("ButtonTrigger::run_if_triggered() is_set: %d\n", is_set);
          if (is_set) {
            // printf("ButtonTrigger::run_if_triggered() button: %d\n", button_);
            bool is_pressing = controller_->get_digital(button_);
            bool was_pressed  = (is_pressing &&
                                !was_triggered_);
            bool was_released = (!is_pressing &&
                                was_triggered_);
            bool is_running = false;
            // for (auto &macro_group : macro_groups) {
            //   is_running = is_running || *macro_group->is_running_ptr;
            // }

            // If the button has been pressed and the task isn't running
            if (was_pressed && !is_running) {
              // Set the function to not run when the button is held
              was_triggered_ = true;
              if (trigger_on_release) return;
            } else if (was_released) {
              was_triggered_ = false;
              if (!trigger_on_release) return;
            } else {
              return;
            }
            // if (run_as_task) {
            // // Run the function in a separate task
            // button_task_t =
            //     pros::c::task_create(task_start_wrapper, (void *)&button_callback,
            //         TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "run_buttons");
            // } else {
            function_();
            // }
          }
        }
      };

      
      public:

      pros::Controller * controller_;
      pros::controller_digital_e_t button_;
      
      ButtonCallback(pros::Controller * controller, pros::controller_digital_e_t button) {
        pressed = new ButtonTrigger(controller, button, false);
        released = new ButtonTrigger(controller, button, true);
      }

      ButtonTrigger *pressed;
      ButtonTrigger *released;

      void run() {
        // printf("ButtonCallback::run() button_: %d\n", button_);
        // printf("ButtonCallback::run() pressed.button_: %d\n", pressed->button_);
        pressed->run_if_triggered();
        released->run_if_triggered();
      }
    };

    pros::Controller * controller_;

    public:
    ControllerCallback(pros::Controller * controller) {
      l1    = new ButtonCallback(controller, pros::E_CONTROLLER_DIGITAL_L1);
      l2    = new ButtonCallback(controller, pros::E_CONTROLLER_DIGITAL_L2);
      r1    = new ButtonCallback(controller, pros::E_CONTROLLER_DIGITAL_R1);
      r2    = new ButtonCallback(controller, pros::E_CONTROLLER_DIGITAL_R2);
      up    = new ButtonCallback(controller, pros::E_CONTROLLER_DIGITAL_UP);
      down  = new ButtonCallback(controller, pros::E_CONTROLLER_DIGITAL_DOWN);
      left  = new ButtonCallback(controller, pros::E_CONTROLLER_DIGITAL_LEFT);
      right = new ButtonCallback(controller, pros::E_CONTROLLER_DIGITAL_RIGHT);
      x     = new ButtonCallback(controller, pros::E_CONTROLLER_DIGITAL_X);
      b     = new ButtonCallback(controller, pros::E_CONTROLLER_DIGITAL_B);
      y     = new ButtonCallback(controller, pros::E_CONTROLLER_DIGITAL_Y);
      a     = new ButtonCallback(controller, pros::E_CONTROLLER_DIGITAL_A);
    }

    ButtonCallback *l1;
    ButtonCallback *l2;
    ButtonCallback *r1;
    ButtonCallback *r2;
    ButtonCallback *up;
    ButtonCallback *down;
    ButtonCallback *left;
    ButtonCallback *right;
    ButtonCallback *x;
    ButtonCallback *b;
    ButtonCallback *y;
    ButtonCallback *a;

    std::vector<ButtonCallback *> button_vector = {l1, l2, r1, r2, up, down, left, right, x, b, y, a};

    void run() {
      for (auto &button_callback : button_vector) {
        button_callback->run();
      }
    }
  };

  pros::Controller * master_controller_;
  pros::Controller * partner_controller_;

  public:

  // pros::Mutex mutex;

  // void interrupt_macro_group(MacroGroup * group) {
  //   mutex.take(TIMEOUT_MAX);
  //   printf("is_running_ptr = %d\r\n", *group->is_running_ptr);
  //   if (*group->is_running_ptr) {
  //     pros::c::task_delete(*group->group_task_t);
  //     printf("DELETED!\r\n");
  //     *group->is_running_ptr = false;
  //   }
  //   mutex.give();
  // }

  ButtonHandler(pros::Controller * master_controller, pros::Controller * partner_controller) {
    master = new ControllerCallback(master_controller);
    partner = new ControllerCallback(partner_controller);
  }

  // ControllerCallback *master;
  ControllerCallback *master;
  ControllerCallback *partner;
  
  void run() {
    master->run();
    partner->run();
  }

};

ButtonHandler button_handler(&master, &partner);

void foo_one() {
  printf("foo_one\n");
}

void foo_two() {
  printf("foo_two\n");
}

void foo_three() {
  printf("foo_three\n");
  button_handler.master->a->pressed->set(&foo_two);
  button_handler.master->a->released->set(&foo_one);
}

void set_callbacks() {
  // MacroGroup test_group;
  button_handler.master->a->pressed->set(&foo_one);
  button_handler.master->a->released->set(&foo_two);
  button_handler.master->b->released->set(&foo_three);
}

void run_buttons() {
  button_handler.run();
}





















bool unnasigned_group;

pros::Mutex mutex;

// Stores what buttons should run which functions
// Is writen to in ::setCalback functions
std::vector<ButtonStruct> button_callbacks;

// void task_start_wrapper(void * void_ptr) {
//   ButtonStruct * struct_ptr = (ButtonStruct*) void_ptr;
//   for(auto group : struct_ptr->macro_groups) {
//     group->is_running_ptr = &struct_ptr->is_running;
//     group->group_task_t = &struct_ptr->button_task_t;
//   }

//   struct_ptr->is_running = true;
//   struct_ptr->function();
//   struct_ptr->is_running = false;
//   mutex.take(TIMEOUT_MAX);
//   mutex.give();
//   printf("ended\r\n");
// }

// void interrupt_macro_group(MacroGroup * group) {
//   mutex.take(TIMEOUT_MAX);
//   printf("is_running_ptr = %d\r\n", *group->is_running_ptr);
//   if (*group->is_running_ptr) {
//     pros::c::task_delete(*group->group_task_t);
//     printf("DELETED!\r\n");
//     *group->is_running_ptr = false;
//   }
//   mutex.give();
// }

// void run_buttons() {
//   // Cycle through all button callbacks
//   for (auto &button_callback : button_callbacks) {
//     bool is_pressing = button_callback.controller->get_digital(button_callback.button);
//     bool was_pressed  = (is_pressing &&
//                          !button_callback.was_triggered);
//     bool was_released = (!is_pressing &&
//                          button_callback.was_triggered);
//     bool is_running = false;
//     for (auto &macro_group : button_callback.macro_groups) {
//       is_running = is_running || *macro_group->is_running_ptr;
//     }

//     // If the button has been pressed and the task isn't running
//     if (was_pressed && !is_running) {
//       // Set the function to not run when the button is held
//       button_callback.was_triggered = true;
//       if (button_callback.trigger_on_release) continue;
//     } else if (was_released) {
//       button_callback.was_triggered = false;
//       if (!button_callback.trigger_on_release) continue;
//     } else {
//       continue;
//     }
//     if (button_callback.run_as_task) {
//     // Run the function in a separate task
//     button_callback.button_task_t =
//         pros::c::task_create(task_start_wrapper, (void *)&button_callback,
//             TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "run_buttons");
//     } else {
//       button_callback.function();
//     }
//   }
// }
} // namespace controllerbuttons
