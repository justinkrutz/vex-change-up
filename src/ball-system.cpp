#include "main.h"

#include "utilities.h"
#include "robot-config.h"
// #include "controller-buttons.h"
// #include "controller-menu.h"
// #include "robot-functions.h"
// #include "auton-from-sd.h"
#include <stdio.h>
#include <complex.h>

// state machine under development
namespace intake {

#define ARM_RANGE 170
double arm_target_pos = 0;

enum class State {
    kError,
    kStowed,
    kReady,
    kRunning,
    kStowedToReady,
    kReadyToStowed,
    kReadyToRunning,
    kRunningToReady,
    kStowedToRunning,
    kRunningToStowed,
    kEjectingBall,
    kEjectingBallToReady};
State state = State::kStowed;

namespace {
  
Slew slew(5);
double target_pct = 0;
double start_of_movement = 0;
pros::motor_brake_mode_e brake_mode = pros::E_MOTOR_BRAKE_BRAKE;
RampMathSettings retract {20, 100, 100, 0.2, 0.2};
RampMathSettings extend {100, 100, 20, 0.2, 0.2};

double nearest_nut_forward(pros::Motor &motor) {
  double pos = motor.get_position();
  return pos - fmod(pos, -45);
}

double nearest_nut_backward(pros::Motor &motor) {
  double pos = motor.get_position();
  return pos - fmod(pos, 45);
}

double nearest_nut(pros::Motor &motor) {
  double pos = motor.get_position();
  if (fmod(pos, 45) < 22.5)
    return pos - fmod(pos, -45);
    return pos - fmod(pos, 45);
}

} // anonymous namespace


namespace actions {

class Action {
 public:
  void trigger() {
    is_new = true;
  }

  bool get_new() {
    if (is_new) {
      is_new = false;
      return true;
    }
    return false;
  }

 private:
  bool is_new = true;
};

Action stow;
Action make_ready;
Action run;

} // namespace actions

namespace statefunctions {
using namespace actions;

void init() {
  intake_left.set_zero_position(-360);
  intake_right.set_zero_position(-360);
}


void any_state() {
}

void stowed() {
  if(run.get_new()) {
    state = State::kStowedToRunning;
  } else if(make_ready.get_new()) {
    state = State::kStowedToReady;
  }
}

void ready() {
  if(stow.get_new()) {
    state = State::kReadyToStowed;
  } else if(run.get_new()) {
    state = State::kReadyToRunning;
  }
}

void running() {
  if(stow.get_new()) {
    state = State::kRunningToStowed;
  } else if(make_ready.get_new()) {
    state = State::kRunningToReady;
  }
}

void stowed_to_ready() {
  // target_pct = 
}

void ready_to_stowed() {
}

void ready_to_running() {
}

void running_to_ready() {
}

void stowed_to_running() {
}

void running_to_stowed() {
}

void ejecting_ball() {
}

void ejecting_ball_to_ready() {
}

} // namespace statefunctions

  void loop() {
    using namespace statefunctions;
    while(true) {
      init();
      any_state();
      switch (state) {
        case State::kStowed:
          stowed();
          break;
        case State::kReady:
          ready();
          break;
        case State::kRunning:
          running();
          break;
        case State::kStowedToReady:
          stowed_to_ready();
          break;
        case State::kReadyToStowed:
          ready_to_stowed();
          break;
        case State::kReadyToRunning:
          ready_to_running();
          break;
        case State::kRunningToReady:
          running_to_ready();
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
        case State::kEjectingBallToReady:
          ejecting_ball_to_ready();
          break;
      }
      intake_left.move_velocity(target_pct * 2);
      intake_right.move_velocity(target_pct * 2);
      intake_left.set_brake_mode(brake_mode);
      intake_right.set_brake_mode(brake_mode);
    }
  }

/*
          switch (current_state) {
            case State::kRetracted:
              break;
            case State::kReady:
              break;
            case State::kRunning:
              break;
            default:
              break;
          }
*/

  void start() {
    pros::Task intake_loop (loop);
  }
}