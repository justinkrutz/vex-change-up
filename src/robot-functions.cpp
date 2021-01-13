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

double nearest_nut(double pos) {
  return pos - fmod(pos, 45);
}

void intake_splay(){
  rollers::intake_queue = 0;
  intake_left.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  intake_right.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  intake_left.move_absolute(nearest_nut(intake_left.get_position() + 20) - 40, 200);
  intake_right.move_absolute(nearest_nut(intake_right.get_position() + 20) - 40, 200);
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




// class SmartMotorController {
  // public:
  SmartMotorController::SmartMotorController(pros::Motor &motor, double timeout_ratio, double slew, int manual_controlls)
      : motor(motor),
      timeout_ratio(timeout_ratio),
      slew(slew) {}

  void SmartMotorController::run() {
    int speed = 0;
    bool all_manuals_are_zero = true;
    for (auto&& manual_speed : all_manual_speeds ) {
      if (manual_speed != 0) {
        all_manuals_are_zero = false;
        break;
      }
    }
    if (all_manuals_are_zero) {
      if (target_queue.size() > 0)  {
        target_queue.front().init_if_new();
        auto_speed = target_queue.front().speed;
        if (target_queue.front().is_complete()) {
          target_queue.pop();
        }
      } else {
        auto_speed = 0;
      }
      speed = auto_speed;
    } else {
      for (auto&& manual_speed : all_manual_speeds ) {
        if (manual_speed != 0) speed = manual_speed;
      }
    }
    motor.move_velocity(slew.new_value(speed) * pct_to_velocity(motor));
  }

  void SmartMotorController::add_target(double target, int speed) {
    target_queue.push(Target(this, target, speed));
  }

  void SmartMotorController::add_target(double target, int speed, double timeout) {
    target_queue.push(Target(this, target, speed, timeout));
  }

  void SmartMotorController::set_manual_speed (int index, int speed) {
    all_manual_speeds[index] = speed;
  }

    SmartMotorController::Target::Target(SmartMotorController *motor_controller, double relative_target, int speed, int timeout)
        : motor_controller(motor_controller),
        relative_target(relative_target),
        speed(speed),
        timeout(timeout) {}
    SmartMotorController::Target::Target(SmartMotorController *motor_controller, double relative_target, int speed)
        : motor_controller(motor_controller),
        relative_target(relative_target),
        speed(speed),
        timeout(auto_timeout()) {}

    void SmartMotorController::Target::init_if_new() {
      if (is_new) {
        is_new = false;
        time_at_start = pros::millis();
        starting_pos = motor_controller->motor.get_position();
      }
    }

    bool SmartMotorController::Target::is_complete() {
      return pros::millis() >= time_at_start + timeout
      || (relative_target > 0 && motor_controller->motor.get_position() >= starting_pos + relative_target)
      || (relative_target <= 0 && motor_controller->motor.get_position() <= starting_pos + relative_target);
    }

    int SmartMotorController::Target::auto_timeout() {
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
      return fabs(relative_target) * gear_ratio;
    }

namespace rollers {
int score_queue = 0;
int intake_queue = 0;
bool top_ball_sensor_last = true;
bool bottom_ball_sensor_last = false;
bool intake_continuous = false;
// bool ball_at_top = true;
bool ball_between_intake_and_rollers = false;
int time_when_bottom_sensor_triggered = 0;
double pos_when_ball_at_top = 0;
SmartMotorController top_roller_smart(top_roller, 1.5, 100, 2);
SmartMotorController bottom_roller_smart(bottom_roller, 1.5, 100, 2);

void main_task() {
  intake_left.set_zero_position(360);
  intake_right.set_zero_position(360);
  intake_left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  intake_right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  bottom_roller.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  top_roller.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  pros::delay(200);
  while (true) {
    bool top_ball_sensor_triggered = !top_ball_sensor_last && top_ball_sensor.get_value() < 2200;
    bool top_ball_sensor_released = top_ball_sensor.get_value() > 2400 && top_ball_sensor_last
                                    && fabs(pos_when_ball_at_top - top_roller.get_position()) > 100;

    bool bottom_ball_sensor_triggered = !bottom_ball_sensor_last && bottom_ball_sensor.get_value() < 2000;
    bool bottom_ball_sensor_released = bottom_ball_sensor.get_value() > 2200 && bottom_ball_sensor_last;

    if (bottom_ball_sensor_triggered) {
      bottom_ball_sensor_last = true;
      bottom_roller_smart.set_manual_speed(0, 100);
    } else if (bottom_ball_sensor_released) {
      bottom_ball_sensor_last = false;
      bottom_roller_smart.set_manual_speed(0, 0);
      bottom_roller_smart.add_target(700, 100);
    }

    if (top_ball_sensor_triggered) {
      top_ball_sensor_last = true;
      top_roller_smart.set_manual_speed(0, 0);
      bottom_roller_smart.set_manual_speed(0, 0);
      // top_roller_smart.add_target(-10, -20);
    } else if (top_ball_sensor_released) {
      top_ball_sensor_last = false;
      top_roller_smart.set_manual_speed(0, 100);
      bottom_roller_smart.set_manual_speed(0, 25);
    }

    if (score_queue > 0) {
      score_queue--;
      bottom_roller_smart.set_manual_speed(0, 0);
      bottom_roller_smart.add_target(500, 100);
      top_roller_smart.add_target(140, 100);
    }

    if (intake_queue > 0) {
      if (bottom_ball_sensor_triggered && intake_queue > 1) {
        intake_queue--;
      } 
      if (bottom_ball_sensor_released && intake_queue == 1) {
        intake_splay();
      } else {
        intake_left.move_velocity(200);
        intake_right.move_velocity(200);
      }
      
    }

    top_roller_smart.run();
    bottom_roller_smart.run();
    pros::delay(5);
  }
}

void score_ball() {
  score_queue++;
}

void add_ball_to_intake_queue() {
  intake_queue++;
  intake_continuous = true;
}

void intake_continuous_false() {
  intake_continuous = false;
}


void bottom_roller_forward() {
  bottom_roller_smart.set_manual_speed(1, 100);
}

void bottom_roller_reverse() {
  bottom_roller_smart.set_manual_speed(1, -100);
}

void rollers_forward() {
  bottom_roller_smart.set_manual_speed(1, 100);
  top_roller_smart.set_manual_speed(1, 100);
}

void rollers_reverse() {
  bottom_roller_smart.set_manual_speed(1, -100);
  top_roller_smart.set_manual_speed(1, -100);
}

void rollers_stop() {
  bottom_roller_smart.set_manual_speed(1, 0);
  top_roller_smart.set_manual_speed(1, 0);
}

} // namespace rollers

controllerbuttons::Macro intakes_back(
    [](){
      rollers::intake_queue = 0;
      left_intake_back.start();
      right_intake_back.start();
    },
    [](){

    },
    {&intake_group},
    {&left_intake_back, &right_intake_back});

/*===========================================================================*/

void set_callbacks() {
  using namespace controllerbuttons;
  using namespace rollers;
  button_handler.master.l1.pressed.set(intake_splay);
  button_handler.master.l2.pressed.set_macro(intakes_back);
  button_handler.master.r1.pressed.set(add_ball_to_intake_queue);
  button_handler.master.r1.released.set(intake_continuous_false);
  button_handler.master.r2.pressed.set(score_ball);
  button_handler.master.down.pressed.set(rollers_reverse);
  button_handler.master.down.released.set(rollers_stop);
  button_handler.master.up.pressed.set(rollers_forward);
  button_handler.master.up.released.set(rollers_stop);
  button_handler.master.x.pressed.set(bottom_roller_forward);
  button_handler.master.x.released.set(rollers_stop);
  button_handler.master.b.pressed.set(bottom_roller_reverse);
  button_handler.master.b.released.set(rollers_stop);

  button_handler.partner.r2.pressed.set(score_ball);
  button_handler.partner.down.pressed.set(rollers_reverse);
  button_handler.partner.down.released.set(rollers_stop);
  button_handler.partner.up.pressed.set(rollers_forward);
  button_handler.partner.up.released.set(rollers_stop);
  button_handler.partner.x.pressed.set(bottom_roller_forward);
  button_handler.partner.x.released.set(rollers_stop);
  button_handler.partner.b.pressed.set(bottom_roller_reverse);
  button_handler.partner.b.released.set(rollers_stop);
}

} // namespace robotfunctions
