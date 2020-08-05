#ifndef CONTROLLER_BUTTONS_H
#define CONTROLLER_BUTTONS_H

namespace controllerbuttons {

#define TASK_SHOULD_END 1 << 31 

struct TaskInterruptedException : public std::exception {
  const char * what () const throw () {
    return "User Terminated";
  }
};

void wait(int wait_time);

class Macro;

struct MacroGroup {
  Macro *macro;
  bool is_running();
  void terminate();
};

class Macro {
  private:
  std::function<void()> function_;
  std::function<void()> clean_up_;
  std::vector<Macro *> macros_ = {this};
  std::vector<MacroGroup *> macro_groups_; 

  std::optional<pros::Task> task_;
  bool is_running_ = false;

  void start_wrapper_();

  public:
  bool is_running();

  std::vector<MacroGroup *> macro_groups();

  void start();

  void terminate();

  Macro(std::function<void()> function,
        std::function<void()> clean_up,
        std::vector<MacroGroup *> macro_groups = {},
        std::vector<Macro *> macros = {});
};


// class ButtonHandler {
//   public:
//   class Controller {
//     public:
//     class Button {
//       public:
//       class Trigger {
//         private:
        
//         pros::Controller * controller_;
//         pros::controller_digital_e_t button_;

//         std::function<void()> function_;
//         std::vector<std::string> button_groups_;
//         std::vector<MacroGroup *> macro_groups_;
        
//         bool was_triggered_ = false;
//         bool is_set_ = false;
//         bool trigger_on_release_;

//         public:
//         Trigger(pros::Controller * controller,
//                 pros::controller_digital_e_t button,
//                 bool trigger_on_release);

//         void set(std::function<void()> function,
//                  std::vector<std::string> button_groups = {},
//                  std::vector<MacroGroup *> macro_groups = {});

//         void set_macro(Macro &macro,
//                        std::vector<std::string> button_groups = {});

//         void clear();

//         void clear_if_in_group(std::string button_group);

//         void run_if_triggered();
//       };

//       Button(pros::Controller * controller,
//              pros::controller_digital_e_t button);

//       Trigger pressed;
//       Trigger released;
//       std::vector<Trigger *> triggers{&pressed, &released};
//     };
    
//     Controller(pros::Controller * controller);

//     Button l1;
//     Button l2;
//     Button r1;
//     Button r2;
//     Button up;
//     Button down;
//     Button left;
//     Button right;
//     Button x;
//     Button b;
//     Button y;
//     Button a;
//     std::vector<Button *> buttons;
//     // std::vector<Button *> buttons{&l1, &l2, &r1, &r2, &up, &down, &left, &right, &x, &b, &y, &a};
//   };


//   ButtonHandler(pros::Controller * master_controller,
//                 pros::Controller * partner_controller);

//   // Controller *master;
//   Controller master;
//   Controller partner;
//   std::vector<Controller *> controllers{&master, &partner};

//   std::vector<Controller::Button::Trigger *> all_triggers;

//   void run();

//   void clear_all();

//   void clear_group(std::string button_group);
// };

// extern ButtonHandler button_handler;

void set_callbacks();

void run_buttons();

} // namespace controllerbuttons

#endif // CONTROLLER_BUTTONS_H
