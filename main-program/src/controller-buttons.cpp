#include "api.h"

#include <bits/stdc++.h>

#include "controller-buttons.h"
#include "robot-config.h"
#include "robot-functions.h"


namespace controllerbuttons {

#define TASK_SHOULD_END 1 << 31 

struct TaskInterruptedException : public std::exception {
  const char * what () const throw () {
    return "User Terminated";
  }
};

void wait(int wait_time) {
  int loop_time;
  if (wait_time > 10) {
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

void count_up_task() {
  printf("start\n");
  for (int i = 0; i < 50; i++) {
    printf("Up %d\n", i);
    wait(20);
  }
}




class Macro;

class MacroGroup {
  public:
  Macro *macro;
  void terminate();
};

class Macro {
  private:
  static std::vector<Macro *> all_macros_; 

  std::function<void()> function_;
  std::function<void()> clean_up_;
  std::vector<Macro *> macros_ = {this}; 
  // std::vector<MacroGroup *> macro_groups = {}; 
  std::vector<std::string> macro_groups_; 

  std::optional<pros::Task> task_;
  bool is_running_ = false;

  void start_wrapper_(){
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

  public:
  static void terminate_macro_group(std::string macro_group){
    for (auto &macro : all_macros_) {
      for (auto &group : macro->macro_groups_) {
        if (group == macro_group) {
          macro->terminate();
          continue;
        }
      }
    }
  }

  bool is_running() {
    return is_running_;
  }

  void start() {
    // for (auto &group : macro_groups) {
    //   group->macro = this;
    // }
    task_ = pros::Task([this](){ start_wrapper_(); }, "Macro");
  }

  void terminate() {
    for (auto &macro : macros_) {
      if (macro->is_running_) {
        macro->task_->notify_ext(TASK_SHOULD_END, pros::E_NOTIFY_ACTION_BITS, NULL);
      }
    }
  }

  Macro(std::function<void()> function,
        std::function<void()> clean_up,
        std::vector<std::string> macro_groups = {},
        std::vector<Macro *> macros = {}) :
        function_(function),
        clean_up_(clean_up),
        macro_groups_(macro_groups){
    macros_.insert(macros_.end(), macros.begin(), macros.end());
    all_macros_.push_back(this);
  }
};

std::vector<Macro *> Macro::all_macros_; 

void MacroGroup::terminate() {
  if (macro) {
    macro->terminate();
  }
}


Macro sub_one(
  [](){
    count_up_task();
  },
  [](){
    printf("clean up sub_one\n");
  }
);

Macro macro_one(
  [](){
    sub_one.start();
  },
  [](){
    printf("clean up macro_one\n");
  },
  {"test"},
  {&sub_one}
);



class ButtonHandler {
  public:
  class Controller {
    public:
    class Button {
      public:
      class Trigger {
        private:
        
        pros::Controller * controller_;
        pros::controller_digital_e_t button_;

        std::function<void()> function_;
        std::vector<std::string> button_groups_;
        std::vector<std::string> macro_groups_;
        
        bool was_triggered_ = false;
        bool is_set_ = false;
        bool trigger_on_release_;

        public:
        Trigger(pros::Controller * controller,
                pros::controller_digital_e_t button,
                bool trigger_on_release) {
          controller_ = controller;
          button_ = button;
          trigger_on_release_ = trigger_on_release;
        }

        void set(std::function<void()> function,
                 std::vector<std::string> button_groups = {},
                 std::vector<std::string> macro_groups = {}) {
          function_ = function;
          button_groups_ = button_groups;
          macro_groups_ = macro_groups_;
          is_set_ = true;
        }

        void set_macro(Macro &macro,
                       std::vector<std::string> button_groups = {}) {
          function_ = [&](){ macro.start(); };
          button_groups_ = button_groups;
          is_set_ = true;
        }

        void clear() {
          is_set_ = false;
        }

        void clear_if_in_group(std::string button_group) {
          for (auto &group_to_clear : button_groups_) {
            if (button_group == group_to_clear){
              clear();
              return;
            }
          }
        }

        void run_if_triggered() {
          if (is_set_) {
            bool is_pressing = controller_->get_digital(button_);
            bool was_pressed  = (is_pressing && !was_triggered_);
            bool was_released = (!is_pressing && was_triggered_);
            bool can_run = true;
            // for (auto &macro_group : macro_groups) {
            //   can_run = can_run && !*macro_group->is_running_ptr;
            // }

            // If the button has been pressed and the task isn't running
            if (was_pressed && can_run) {
              // Set the function to not run when the button is held
              was_triggered_ = true;
              if (trigger_on_release_) return;
            } else if (was_released) {
              was_triggered_ = false;
              if (!trigger_on_release_) return;
            } else {
              return;
            }
            function_();
          }
        }
      };

      Button(pros::Controller * controller,
             pros::controller_digital_e_t button) :
             pressed(controller, button, false),
             released(controller, button, true) {}

      Trigger pressed;
      Trigger released;
      std::vector<Controller::Button::Trigger *> triggers{&pressed, &released};
    };
    
    Controller(pros::Controller * controller) :
               l1    (controller, pros::E_CONTROLLER_DIGITAL_L1),
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

    Button l1;
    Button l2;
    Button r1;
    Button r2;
    Button up;
    Button down;
    Button left;
    Button right;
    Button x;
    Button b;
    Button y;
    Button a;
    std::vector<Button *> buttons{&l1, &l2, &r1, &r2, &up, &down, &left, &right, &x, &b, &y, &a};
  };


  ButtonHandler(pros::Controller * master_controller,
                pros::Controller * partner_controller) :
                master(master_controller), 
                partner(partner_controller) {
    for (auto &controller : controllers) {
      for (auto &button : controller->buttons) {
        for (auto &trigger : button->triggers) { 
          all_triggers.push_back(trigger);
        }
      }
    }
  }

  // Controller *master;
  Controller master;
  Controller partner;
  std::vector<Controller *> controllers{&master, &partner};

  std::vector<Controller::Button::Trigger *> all_triggers;

  void run() {
    for (auto &trigger : all_triggers) {
      trigger->run_if_triggered();
    }
  }

  void clear_all() {
    for (auto &trigger : all_triggers) {
      trigger->clear();
    }
  }

  void clear_group(std::string button_group) {
    for (auto &trigger : all_triggers) {
      trigger->clear_if_in_group(button_group);
    }
  }
};

ButtonHandler button_handler(&master, &partner);



void set_callbacks() {
  button_handler.master.a.pressed.set_macro(macro_one, {"test"});
  button_handler.master.b.pressed.set_macro(macro_one, {"test"});
  button_handler.master.b.released.set([&](){ macro_one.terminate(); }, {"test"});
  button_handler.master.y.pressed.set([&](){ macro_one.terminate(); }, {"test"});
  button_handler.master.x.pressed.set([&](){ Macro::terminate_macro_group("test"); }, {"test"});
}

void run_buttons() {
  button_handler.run();
}

} // namespace controllerbuttons