#include "main.h"

#include "utilities.h"
#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
// #include "robot-functions.h"
// #include "auton-from-sd.h"
#include <stdio.h>
#include <complex.h>

// state machine under development
namespace ballsystem {

class IntakeStateMachine{
 public:
  IntakeStateMachine(pros::Motor &motor) : motor(motor), slew(5.0) {}

  enum class Action {
    kNone,
    kStow,
    kSplay,
    kRun,
    kEjectBall
  };
  static Action target_action;
  void set_action(Action action) {
    target_action = action;
  }
  bool get_new_action(Action action) {
    if (target_action == action) {
      target_action == Action::kNone;
      return true;
    }
    return false;
  }

  pros::Motor &motor;
  enum class State {
    kError,
    kStowed,
    kSplayed,
    kRunning,
    kStowedToSplayed,
    kSplayedToStowed,
    kSplayedToRunning,
    kRunningToSplayed,
    kStowedToRunning,
    kRunningToStowed,
    kEjectingBall,
    kEjectingBallToSplayed
  };
  State state = State::kStowed;

  Slew slew;
  double target_pct = 0;
  double target_pos = 360;
  RampMathSettings retract_settings {20, 100, 100, 0.2, 0.2};
  RampMathSettings extend_settings {100, 100, 20, 0.2, 0.2};

  double nearest_nut(double pos) {
    return pos - fmod(pos, 45);
  }

  void hold_position(double pos) {
    target_pct = pos - motor.get_position();
  }

  void any_state() {
  }

  void stowed() {
    // if(target_action == Action::kRun) {
    //   state = State::kStowedToRunning;
    // } else 
    if(target_action == Action::kSplay) {
      state = State::kStowedToSplayed;
    }
    // hold_position(target_pos);
    motor.move_absolute(target_pos, 200);
    // target_pct = 0;
  }

  void splayed() {
    if(target_action == Action::kStow) {
      state = State::kSplayedToStowed;
    }
    // else if(target_action == Action::kRun) {
    //   state = State::kSplayedToRunning;
    // }
    // hold_position(target_pos);
    motor.move_absolute(target_pos, 200);
    // target_pct = 0;
  }

  void running() {
    // if(target_action == Action::kStow) {
    //   state = State::kRunningToStowed;
    // } else if(target_action == Action::kSplay) {
    //   state = State::kRunningToSplayed;
    // }
  }

  void stowed_to_splayed() {
    if(target_action == Action::kSplay) {
      target_action = Action::kNone;
      target_pos = motor.get_position() + 135;
    }
    if(motor.get_position() >= target_pos) state = State::kSplayed;
    // target_pct = rampMath(target_pos - motor.get_position(), 135, extend_settings);
    motor.move_absolute(target_pos, 200);
  }

  void splayed_to_stowed() {
    if(target_action == Action::kStow) {
      target_action = Action::kNone;
      target_pos = nearest_nut(motor.get_position() + 135) - 135;
    }
    if(motor.get_position() <= target_pos) state = State::kStowed;
    // target_pct = rampMath(target_pos - motor.get_position(), -135, retract_settings);
    motor.move_absolute(target_pos, 200); 
  }

  void splayed_to_running() {
  }

  void running_to_splayed() {
  }

  void stowed_to_running() {
  }

  void running_to_stowed() {
  }

  void ejecting_ball() {
  }

  void ejecting_ball_to_splayed() {
  }

  void loop() {
    motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    while(true) {
      any_state();
      switch (state) {
        case State::kStowed:
          stowed();
          break;
        case State::kSplayed:
          splayed();
          break;
        case State::kRunning:
          running();
          break;
        case State::kStowedToSplayed:
          stowed_to_splayed();
          break;
        case State::kSplayedToStowed:
          splayed_to_stowed();
          break;
        case State::kSplayedToRunning:
          splayed_to_running();
          break;
        case State::kRunningToSplayed:
          running_to_splayed();
          break;
        case State::kStowedToRunning:
          stowed_to_running();
          break;
        case State::kRunningToStowed:
          running_to_stowed();
          break;
        case State::kEjectingBall:
          ejecting_ball();
          break;
        case State::kEjectingBallToSplayed:
          ejecting_ball_to_splayed();
          break;
      }
      // motor.move_velocity(slew.new_value(target_pct * 2));
      pros::delay(10);
    }
  }

  void init() {
    motor.set_zero_position(360);
    pros::Task intake_loop ([this](){ loop(); });
  }
};

IntakeStateMachine::Action IntakeStateMachine::target_action = Action::kNone;

IntakeStateMachine left_intake_sm(intake_left);
IntakeStateMachine right_intake_sm(intake_right);

void set_action_stow() {
  IntakeStateMachine::target_action = IntakeStateMachine::Action::kStow;
}

void set_action_splay() {
  IntakeStateMachine::target_action = IntakeStateMachine::Action::kSplay;
}

void init() {
  left_intake_sm.init();
  // right_intake_sm.init();
}

void set_callbacks() {
  using namespace controllerbuttons;
  button_handler.master.l1.pressed.set(set_action_splay);
  button_handler.master.l2.pressed.set(set_action_stow);
}

void debug() {
  controllermenu::master_print_array[0] = "target_action: " + std::to_string(static_cast<int>(IntakeStateMachine::target_action));
  controllermenu::master_print_array[1] = "left_intake_sm: " + std::to_string(static_cast<int>(left_intake_sm.state));
  controllermenu::master_print_array[3] = "right_intake_sm: " + std::to_string(static_cast<int>(right_intake_sm.state));
}

} // namespace ballsystem