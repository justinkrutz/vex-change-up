#include "api.h"

#include <bits/stdc++.h>

#include "controller-buttons.h"
#include "robot-config.h"

namespace controllerbuttons {

void wait(int wait_time) {
  int loop_time;
  if (wait_time < 10) {
    loop_time = wait_time;
  } else {
    loop_time = 10;
  }
  for (int i = 0; i < wait_time / loop_time; i++) {
    if (pros::c::task_notify_take(false, 0) & TASK_SHOULD_END) {
      throw TaskInterruptedException();
    }
    pros::delay(loop_time);
  }
}



void Macro::start_wrapper_(){
  is_running_ = true;
  try {
    function_();
  }
  catch (TaskInterruptedException& e) {
    std::cout << e.what() << std::endl;
  }
  clean_up_();
  is_running_ = false;
}

bool Macro::is_running() {
  for (auto &macro : macros_) {
    if (macro->is_running_) {
      return true;
    }
  }
  return false;
}

std::vector<MacroGroup *> Macro::macro_groups() {
  return macro_groups_;
}

void Macro::start() {
  for (auto &group : macro_groups_) {
    group->terminate();
  }
  
  for (auto &group : macro_groups_) {
    group->macro = this;
  }
  task_ = pros::Task([this](){ start_wrapper_(); }, "Macro");
}

void Macro::terminate() {
  for (auto &macro : macros_) {
    if (macro->is_running_) {
      macro->task_->notify_ext(TASK_SHOULD_END, pros::E_NOTIFY_ACTION_BITS, NULL);
    }
  }
}

Macro::Macro(std::function<void()> function,
      std::function<void()> clean_up,
      std::vector<MacroGroup *> macro_groups,
      std::vector<Macro *> macros) :
      function_(function),
      clean_up_(clean_up),
      macro_groups_(macro_groups){
  macros_.insert(macros_.end(), macros.begin(), macros.end());
}

void MacroGroup::terminate() {
  if (macro) {
    macro->terminate();
  }
}

bool MacroGroup::is_running() {
  if (macro) {
    return macro->is_running();
  }
  return false;
}



ButtonHandler::Controller::Button::Trigger::Trigger(pros::Controller * controller,
        pros::controller_digital_e_t button,
        bool trigger_on_release) {
  controller_ = controller;
  button_ = button;
  trigger_on_release_ = trigger_on_release;
}

void ButtonHandler::Controller::Button::Trigger::set(std::function<void()> function,
          std::vector<std::string> button_groups,
          std::vector<MacroGroup *> macro_groups) {
  function_ = function;
  button_groups_ = button_groups;
  macro_groups_ = macro_groups_;
  is_set_ = true;
}

void ButtonHandler::Controller::Button::Trigger::set_macro(Macro &macro,
                std::vector<std::string> button_groups) {
  function_ = [&](){ macro.start(); };
  macro_groups_ = macro.macro_groups();
  button_groups_ = button_groups;
  is_set_ = true;
}

void ButtonHandler::Controller::Button::Trigger::clear() {
  is_set_ = false;
}

void ButtonHandler::Controller::Button::Trigger::clear_if_in_group(std::string button_group) {
  for (auto &group_to_clear : button_groups_) {
    if (button_group == group_to_clear){
      clear();
      return;
    }
  }
}

void ButtonHandler::Controller::Button::Trigger::run_if_triggered() {
  if (is_set_) {
    bool is_pressing = controller_->get_digital(button_);
    bool was_pressed  = (is_pressing && !was_triggered_);
    bool was_released = (!is_pressing && was_triggered_);
    bool can_run = true;
    for (auto &group : macro_groups_) {
      can_run = can_run && !group->is_running();;
    }

    // If the button has been pressed and the task isn't running
    if (was_pressed && can_run) {
      // Set the function to not run when the button is held
      was_triggered_ = true;
      if (!trigger_on_release_) {
        function_();
      }
    } else if (was_released && can_run) {
      was_triggered_ = false;
      if (trigger_on_release_) {
        function_();
      }
    }
  }
}


ButtonHandler::Controller::Button::Button(
    pros::Controller * controller, 
    pros::controller_digital_e_t button)
    : pressed(controller, button, false),
      released(controller, button, true) {}

ButtonHandler::Controller::Controller(pros::Controller * controller)
    : l1    (controller, pros::E_CONTROLLER_DIGITAL_L1),
      l2    (controller, pros::E_CONTROLLER_DIGITAL_L2),
      r1    (controller, pros::E_CONTROLLER_DIGITAL_R1),
      r2    (controller, pros::E_CONTROLLER_DIGITAL_R2),
      up    (controller, pros::E_CONTROLLER_DIGITAL_UP),
      down  (controller, pros::E_CONTROLLER_DIGITAL_DOWN),
      left  (controller, pros::E_CONTROLLER_DIGITAL_LEFT),
      right (controller, pros::E_CONTROLLER_DIGITAL_RIGHT),
      x     (controller, pros::E_CONTROLLER_DIGITAL_X),
      b     (controller, pros::E_CONTROLLER_DIGITAL_B),
      y     (controller, pros::E_CONTROLLER_DIGITAL_Y),
      a     (controller, pros::E_CONTROLLER_DIGITAL_A) {}

ButtonHandler::ButtonHandler(
    pros::Controller * master_controller,
    pros::Controller * partner_controller)
    : master(master_controller), 
      partner(partner_controller) {
  for (auto &controller : controllers) {
    for (auto &button : controller->buttons) {
      for (auto &trigger : button->triggers) { 
        all_triggers.push_back(trigger);
      }
    }
  }
}

void ButtonHandler::run() {
  for (auto &trigger : all_triggers) {
    trigger->run_if_triggered();
  }
}

void ButtonHandler::clear_all() {
  for (auto &trigger : all_triggers) {
    trigger->clear();
  }
}

void ButtonHandler::clear_group(std::string button_group) {
  for (auto &trigger : all_triggers) {
    trigger->clear_if_in_group(button_group);
  }
}

ButtonHandler button_handler(&master, &partner);

void run_buttons() {
  button_handler.run();
}

} // namespace controllerbuttons