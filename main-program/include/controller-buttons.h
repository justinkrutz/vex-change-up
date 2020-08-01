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

// class ButtonHandler {
//   private:
//   class ControllerCallback {
//     private:
//     class ButtonCallback {
//       private:
//       class ButtonTrigger {
//         private:
//         public:
//         pros::Controller * controller_;
//         pros::controller_digital_e_t button_;

//         std::function<void()> function_;
//         bool was_triggered_ = false;
//         bool is_set = false;
//         bool trigger_on_release;
//         ButtonTrigger(pros::Controller * controller, pros::controller_digital_e_t button, bool trigger_on_release_arg);
        
//         void set(std::function<void()> function);

//         void clear();

//         void run_if_triggered();
//       };
//       public:

//       pros::Controller * controller_;
//       pros::controller_digital_e_t button_;
      
//       ButtonCallback(pros::Controller * controller, pros::controller_digital_e_t button) {
//         pressed = new ButtonTrigger(controller, button, false);
//         released = new ButtonTrigger(controller, button, true);
//       }

//       ButtonTrigger *pressed;
//       ButtonTrigger *released;

//       void run();
//     };

//     pros::Controller * controller_;

//     public:
//     ControllerCallback(pros::Controller * controller);

//     ButtonCallback *l1;
//     ButtonCallback *l2;
//     ButtonCallback *r1;
//     ButtonCallback *r2;
//     ButtonCallback *up;
//     ButtonCallback *down;
//     ButtonCallback *left;
//     ButtonCallback *right;
//     ButtonCallback *x;
//     ButtonCallback *b;
//     ButtonCallback *y;
//     ButtonCallback *a;

//     std::vector<ButtonCallback *> button_vector = {l1, l2, r1, r2, up, down, left, right, x, b, y, a};

//     void run();
//   };

//   pros::Controller * master_controller_;
//   pros::Controller * partner_controller_;

//   public:
//   ButtonHandler(pros::Controller * master_controller, pros::Controller * partner_controller);
//   ControllerCallback *master;
//   ControllerCallback *partner;
  
//   void run();
// };

// extern ButtonHandler button_handler;









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
