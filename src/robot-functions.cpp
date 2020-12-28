#include "main.h"

#include "utilities.h"
#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"
#include "auton-from-sd.h"
#include <stdio.h>
#include <complex.h>

namespace robotfunctions {

controllerbuttons::MacroGroup intake_group;

// state machine under development
namespace intake {
  #define ARM_RANGE 170
  double arm_target_pos = 0;

  enum class State {kRetracted, kReady, kRunning};
  State target_state = State::kRetracted;
  State current_state = State::kRetracted;

  double target_pct = 0;
  pros::motor_brake_mode_e brake_mode = pros::E_MOTOR_BRAKE_BRAKE;

  void loop() {
    while(true) {

      switch (target_state) {
        case State::kRetracted:
          switch (current_state) {
            case State::kReady:
              // break;
            case State::kRunning:
              break;
          }
          break;
        case State::kReady:
          switch (current_state) {
            case State::kRetracted:
              break;
            case State::kRunning:
              target_pct = 0;
              break;
          }
          break;
        case State::kRunning:
          switch (current_state) {
            case State::kRetracted:
              // break;
            case State::kReady:
              target_pct = 100;
              break;
          }
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


bool intakes_retracted = false;
// bool intake_sensors_last = true;
int intakes_retracted_time = 0;

bool intakes_extended() {
  // bool intake_sensors = right_intake_sensor.get_value() < 20;
  bool result = !intakes_retracted;
      // || pros::millis() - intakes_retracted_time > 700
      // || (intake_sensors_last && !intake_sensors && intake_right.get_target_velocity() > 10);
  // intake_sensors_last = intake_sensors;
  // intakes_retracted = !result;
  return result;
}


void intake_back(pros::Motor &motor) {
  motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  double start_pos = motor.get_position();
  int start_time = pros::millis();
  while (pros::millis() - start_time < 600) {
    motor.move_velocity(MIN(-(motor.get_position() - start_pos) * 1.7 - 200, -10));
    controllerbuttons::wait(10);
  }
  motor.move_velocity(0);
}

controllerbuttons::Macro left_intake_back(
    [](){
      intake_back(intake_left);
    },
    [](){
      intake_left.move_velocity(0);
    },
    {&intake_group});

controllerbuttons::Macro right_intake_back(
    [](){
      intake_back(intake_right);
    },
    [](){
      intake_right.move_velocity(0);
    },
    {&intake_group});




class SmartMotorController {
  public:
  SmartMotorController(pros::Motor &motor, double timeout_ratio, double slew) : motor(motor), timeout_ratio(timeout_ratio), slew(slew) {}

  pros::Motor &motor;
  double timeout_ratio;
  Slew slew;

  void run() {
    if (target_queue.size() < 1) {
      motor.move_velocity(slew.new_value(manual_speed * pct_to_velocity(motor)));
    } else{
      manual_speed = 0;
      motor.move_velocity(slew.new_value(target_queue.back().speed * pct_to_velocity(motor)));
      if (target_queue.back().is_complete()) {
        target_queue.pop();
        target_queue.back().init_if_new();
      }
    }
  }
  void add_target(double target, int speed) {
    target_queue.push(Target(this, target, speed));
  }

  void add_target(double target, int speed, double timeout) {
    target_queue.push(Target(this, target, speed, timeout));
  }

  double manual_speed = 0;

  class Target {
    public:
    Target(SmartMotorController *motor_controller, double relative_target, int speed, int timeout)
        : motor_controller(motor_controller),
        relative_target(relative_target),
        speed(speed),
        timeout(timeout) {}
    Target(SmartMotorController *motor_controller, double relative_target, int speed)
        : motor_controller(motor_controller),
        relative_target(relative_target),
        speed(speed),
        timeout(auto_timeout()) {}

    void init_if_new() {
      if (is_new) {
        time_at_start = pros::millis();
      }
    }

    bool is_complete() {
      return motor_controller->motor.get_position() >= starting_pos + relative_target
          || pros::millis() >= time_at_start + timeout;
    }

    SmartMotorController *motor_controller;
    double relative_target;
    int speed;
    bool is_new = true;
    int auto_timeout() {
      int gear_ratio;
      switch (motor_controller->motor.get_gearing()) {
        case pros::E_MOTOR_GEARSET_36:
          gear_ratio = DEG_PER_SEC/36;
          break;
        case pros::E_MOTOR_GEARSET_06:
          gear_ratio = DEG_PER_SEC/06;
          break;
        case pros::E_MOTOR_GEARSET_18:
        default:
          gear_ratio = DEG_PER_SEC/18;
          break;
      }
      return relative_target * gear_ratio;
    }
    int timeout;

    int time_at_start;
    double starting_pos;
  };
  std::queue<Target> target_queue = {};
};

namespace rollers {
int score_queue = 0;
int intake_queue = 0;
int balls_in_robot = 0;
bool top_ball_sensor_last = true;
bool bottom_ball_sensor_last = true;
bool intake_continuous = false;
SmartMotorController top_roller_smart(top_roller, 1.5, 6);
SmartMotorController bottom_roller_smart(bottom_roller, 1.5, 6);

void main_task() {
  intake_left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  intake_right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  bottom_roller.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  top_roller.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  pros::delay(500);
  while (true) {
    bool top_ball_sensor_pressed = !top_ball_sensor_last && top_ball_sensor.get_value();
    bool top_ball_sensor_released = top_ball_sensor_last && !top_ball_sensor.get_value();
    bool bottom_ball_sensor_triggered = intakes_extended() && !bottom_ball_sensor_last && bottom_ball_sensor.get_value() < 2000;
    // controllermenu::master_print_array[0] = "BIR: " + std::to_string(balls_in_robot);
    // controllermenu::master_print_array[1] = "IR: " + std::to_string(intakes_retracted);
    // controllermenu::master_print_array[2] = "LS: " + std::to_string(bottom_ball_sensor.get_value());
    // controllermenu::master_print_array[0] = "BIR: " + std::to_string(balls_in_robot) + " SQ: " + std::to_string(score_queue) + " IQ: " + std::to_string(intake_queue);
    // controllermenu::master_print_array[1] = "IR: " + std::to_string(intakes_retracted);
    // controllermenu::master_print_array[2] = "LLS: " + std::to_string(left_intake_sensor.get_value()) + " RLS: " + std::to_string(right_intake_sensor.get_value());
    controllermenu::partner_print_array[0] = "BIR: " + std::to_string(balls_in_robot) + " SQ: " + std::to_string(score_queue) + " IQ: " + std::to_string(intake_queue);
    controllermenu::partner_print_array[1] = "IR: " + std::to_string(intakes_retracted);
    // controllermenu::partner_print_array[2] = "LLS: " + std::to_string(left_intake_sensor.get_value()) + " RLS: " + std::to_string(right_intake_sensor.get_value());
    if (bottom_ball_sensor_triggered) {
      bottom_ball_sensor_last = true;
      if (balls_in_robot < 3) {
      balls_in_robot++;
      }
      switch (balls_in_robot) {
        case 1:
          // top_roller.move_relative(1000, 300);
          top_roller_smart.add_target(1000, 50);
          // bottom_roller.move_relative(750, 600);
          bottom_roller_smart.add_target(750, 100);
          break;
        case 2:
          // bottom_roller.move_relative(750, 600);
          bottom_roller_smart.add_target(750, 100);
          break;
        case 3:
          // bottom_roller.move_relative(270, 600);
          bottom_roller_smart.add_target(270, 100);
          break;
      }
    } else if (bottom_ball_sensor.get_value() > 2200 && bottom_ball_sensor_last) {
      bottom_ball_sensor_last = false;
    }
    if (top_ball_sensor_pressed) {
      top_ball_sensor_last = true;
    } else if (top_ball_sensor_released) {
      top_ball_sensor_last = false;
    }
    if (score_queue > 0) {
      balls_in_robot--;
      score_queue--;
      // top_roller.move_absolute(top_roller.get_target_position() + 500, 600);
      // bottom_roller.move_absolute(bottom_roller.get_target_position() + 500, 600);
      bottom_roller_smart.add_target(500, 100);
      top_roller_smart.add_target(500, 100);
      
      pros::delay(500);
    }
    // if (top_ball_sensor_released && ) {
    //   balls_in_robot--;
    // }
    if (intake_queue > 0 || intake_continuous) {
      intake_left.move_velocity(200);
      intake_right.move_velocity(200);
      if (bottom_ball_sensor_triggered) {
        pros::delay(200);
        if (intake_queue > 0) {
          intake_queue--;
        }
        intake_left.move_relative(-30, 200);
        intake_right.move_relative(-30, 200);
      }
    }

    top_roller_smart.run();
    bottom_roller_smart.run();
    pros::delay(5);
  }
}

void score_ball() {
  if (balls_in_robot > 0) {
    score_queue++;
  }
}

void add_ball_to_intake_queue() {
  intake_queue++;
  intake_continuous = true;
}

void intake_continuous_false() {
  intake_continuous = false;
}


void top_roller_forward() {
  top_roller_smart.manual_speed = 100;
}

void top_roller_reverse() {
  top_roller_smart.manual_speed = -100;
}

void rollers_forward() {
  bottom_roller_smart.manual_speed = 100;
  top_roller_smart.manual_speed = 100;
}

void rollers_reverse() {
  bottom_roller_smart.manual_speed = -100;
  top_roller_smart.manual_speed = -100;
}

void rollers_stop() {
  bottom_roller_smart.manual_speed = 0;
  top_roller_smart.manual_speed = 0;
}

} // namespace rollers

void intake_toggle(){
  rollers::intake_queue = 0;
  intake_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  intake_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  rollers::intake_queue = 0;
  intake_left.move_relative(-40, 200);
  intake_right.move_relative(-40, 200);
}

controllerbuttons::Macro intakes_back(
    [](){
      // intakes_retracted = true;
      intakes_retracted_time = pros::millis();
      rollers::intake_queue = 0;
      left_intake_back.start();
      right_intake_back.start();
    },
    [](){

    },
    {&intake_group},
    {&left_intake_back, &right_intake_back});



void add_ball_to_robot() {
  rollers::balls_in_robot++;
}

void remove_ball_from_robot() {
  rollers::balls_in_robot--;
}

void set_retracted_false() {
  intakes_retracted = false;
}

/*===========================================================================*/

void set_callbacks() {
  using namespace controllerbuttons;
  using namespace rollers;
  button_handler.master.l1.pressed.set(intake_toggle);
  button_handler.master.l2.pressed.set_macro(intakes_back);
  button_handler.master.r1.pressed.set(add_ball_to_intake_queue);
  button_handler.master.r1.released.set(intake_continuous_false);
  button_handler.master.r2.pressed.set(score_ball);
  // button_handler.master.down.pressed.set(top_roller_reverse);
  button_handler.master.down.pressed.set(rollers_reverse);
  button_handler.master.down.released.set(rollers_stop);
  // button_handler.master.up.pressed.set(top_roller_forward);
  button_handler.master.up.pressed.set(rollers_forward);
  button_handler.master.up.released.set(rollers_stop);

  button_handler.partner.r2.pressed.set(score_ball);
  button_handler.partner.down.pressed.set(rollers_reverse);
  button_handler.partner.down.released.set(rollers_stop);
  button_handler.partner.up.pressed.set(rollers_forward);
  button_handler.partner.up.released.set(rollers_stop);
  button_handler.partner.left.pressed.set(remove_ball_from_robot);
  button_handler.partner.right.pressed.set(add_ball_to_robot);
  button_handler.partner.b.pressed.set(set_retracted_false);

}

} // namespace robotfunctions
