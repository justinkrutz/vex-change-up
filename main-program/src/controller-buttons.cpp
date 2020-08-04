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

void wait(int wait_time, int loop_time = 10) {
  for (int i = 0; i < wait_time / loop_time; i++) {
    if (pros::c::task_notify_take(false, 0) & TASK_SHOULD_END) {
      throw TaskInterruptedException();
    }
    pros::delay(loop_time);
  }
}

void count_up_task() {
  printf("start\n");
  int count = 0;

  for (int i = 0; i < 50; i++) {
    printf("Up %d\n", count ++);
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
  // std::vector<MacroGroup *> macro_groups; 
  std::optional<pros::Task> task_;

  void start_wrapper(){
    is_running = true;
    try {
      function();
    }
    catch (TaskInterruptedException& e) {
      std::cout << "TaskInterruptedException caught" << std::endl;
      std::cout << e.what() << std::endl;
    }
    clean_up();
    is_running = false;
  }

  protected:
  virtual void function() {}
  virtual void clean_up() {}
  
  public:
  bool is_running = false;

  void start() {
    // for (auto &group : macro_groups) {
    //   group->macro = this;
    // }
    task_ = pros::Task([this](){ start_wrapper(); }, "Macro");
  }

  void terminate() {
    if (is_running) {
      task_->notify_ext(TASK_SHOULD_END, pros::E_NOTIFY_ACTION_BITS, NULL);
      printf("terminated\n");
    }
  }
};

void MacroGroup::terminate() {
  if (macro) {
    macro->terminate();
  }
}




// class MacroBase {
//   private:

//   pros::Mutex mutex;
//   pros::Task task_;

//   void start_wrapper(){
//     is_running = true;
//     function();
//     is_running = false;
//     mutex.take(TIMEOUT_MAX);
//     mutex.give();
//   }

//   protected:
//   virtual void function() {}
//   virtual void clean_up() {}
  
//   public:
//   bool is_running = false;

//   void start() {
//     mutex.take(TIMEOUT_MAX);
//     task_ = pros::Task([this](){ start_wrapper(); }, "Macro");
//     mutex.give();
//   }

//   void terminate() {
//     mutex.take(TIMEOUT_MAX);
//     // for (auto &task : tasks) {
//     if (is_running) {
//       task_.remove();
//       is_running = false;
//       printf("terminated\n");
//     }
//     mutex.give();
//     clean_up();
//   }
// };






// class SubMacro {
//   private:
//   pros::Task *task_;
//   bool is_running;

//   public:
//   void start() {
    
//   }

//   void end() {

//   }

//   void wait_for_completion() {
//     while (is_running) {
//       pros::delay(10);
//     }
//   }
// };



// class : public Macro {
//     class : public SubMacro {
//     void function() {
//       // drive
//     }
    
//     void clean_up() {
//     }
//   } drive(this);

//   void function() {
//     // do stuff
//     drive.start();
//     // do other stuff
//     drive.wait_for_completion();
//     // do even more stuff
//   }
  
//   void clean_up() {
//   }
// } score;

class MacroOne : public Macro {
  void function() {
    count_up_task();
  }
  
  void clean_up() {
    // clean up stuff
  }
};

MacroOne macro_one;







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
        bool run_as_task_;
        
        bool was_triggered_ = false;
        // bool is_running;
        bool is_set_ = false;
        bool trigger_on_release_;
        // std::vector<MacroGroup *> macro_groups;

        public:
        Trigger(pros::Controller * controller,
                pros::controller_digital_e_t button,
                bool trigger_on_release) {
          controller_ = controller;
          button_ = button;
          trigger_on_release_ = trigger_on_release;
        }

        void set(std::function<void()> function,
                 std::vector<std::string> button_groups = {"default"},
                 bool run_as_task = false) {
          function_ = function;
          button_groups_ = button_groups;
          run_as_task_ = run_as_task;
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






void foo_one() {
  printf("foo_one\n");
  macro_one.start();
  pros::delay(200);
  macro_one.terminate();
  pros::delay(2000);
  macro_one.terminate();
}

void set_callbacks() {
  button_handler.master.a.pressed.set(foo_one, {"test"});
  button_handler.master.b.pressed.set([&](){ macro_one.start(); }, {"test"});
  button_handler.master.b.released.set([&](){ macro_one.terminate(); }, {"test"});
  button_handler.master.y.pressed.set([&](){ macro_one.terminate(); }, {"test"});
}

void run_buttons() {
  // macro_one.start();
  // pros::delay(200);
  // macro_one.terminate();
  button_handler.run();
}

} // namespace controllerbuttons