#include "main.h"

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


bool intakes_retracted = true;
bool intake_sensors_last = true;
int intakes_retracted_time = 0;

bool intakes_extended() {
  bool intake_sensors = right_intake_sensor.get_value() < 20;
  bool result = !intakes_retracted
      || pros::millis() - intakes_retracted_time > 700
      || (intake_sensors_last && !intake_sensors && intake_right.get_target_velocity() > 10);
  intake_sensors_last = intake_sensors;
  intakes_retracted = !result;
  return result;
}


void intake_back(pros::Motor &motor, pros::ADIAnalogIn &sensor) {
  motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  double start_pos = motor.get_position();
  int start_time = pros::millis();
  while (sensor.get_value() > 20 && pros::millis() - start_time < 800) {
    motor.move_velocity(MIN(-(motor.get_position() - start_pos) * 1.7 - 200, -10));
    controllerbuttons::wait(10);
  }
  motor.move_velocity(0);
}

controllerbuttons::Macro left_intake_back(
    [](){
      intake_back(intake_left, left_intake_sensor);
    },
    [](){
      intake_left.move_velocity(0);
    },
    {&intake_group});

controllerbuttons::Macro right_intake_back(
    [](){
      intake_back(intake_right, right_intake_sensor);
    },
    [](){
      intake_right.move_velocity(0);
    },
    {&intake_group});



void top_roller_forward() {
  top_roller.move_velocity(600);
}

void top_roller_reverse() {
  top_roller.move_velocity(-600);
}

void rollers_forward() {
  bottom_roller.move_velocity(600);
  top_roller.move_velocity(600);
}

void rollers_reverse() {
  bottom_roller.move_velocity(-600);
  top_roller.move_velocity(-600);
}

void rollers_stop() {
  bottom_roller.move_velocity(0);
  top_roller.move_velocity(0);
}

namespace rollers {
  int score_queue = 0;
  int intake_queue = 0;
  int balls_in_robot = 0;
  bool ball_sensor_last = true;

  void main_task() {
    intake_left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    intake_right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    bottom_roller.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    top_roller.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    pros::delay(500);
    while (true) {
      bool ball_sensor_triggered = intakes_extended() && !ball_sensor_last && ball_sensor.get_value() < 2000;
      // controllermenu::master_print_array[0] = "BIR: " + std::to_string(balls_in_robot);
      // controllermenu::master_print_array[1] = "IR: " + std::to_string(intakes_retracted);
      // controllermenu::master_print_array[2] = "LS: " + std::to_string(ball_sensor.get_value());
      // controllermenu::master_print_array[0] = "BIR: " + std::to_string(balls_in_robot) + " SQ: " + std::to_string(score_queue) + " IQ: " + std::to_string(intake_queue);
      // controllermenu::master_print_array[1] = "IR: " + std::to_string(intakes_retracted);
      // controllermenu::master_print_array[2] = "LLS: " + std::to_string(left_intake_sensor.get_value()) + " RLS: " + std::to_string(right_intake_sensor.get_value());
      controllermenu::partner_print_array[0] = "BIR: " + std::to_string(balls_in_robot) + " SQ: " + std::to_string(score_queue) + " IQ: " + std::to_string(intake_queue);
      controllermenu::partner_print_array[1] = "IR: " + std::to_string(intakes_retracted);
      controllermenu::partner_print_array[2] = "LLS: " + std::to_string(left_intake_sensor.get_value()) + " RLS: " + std::to_string(right_intake_sensor.get_value());
      if (ball_sensor_triggered) {
        ball_sensor_last = true;
        if (balls_in_robot < 3) {
        balls_in_robot++;
        }
        switch (balls_in_robot) {
          case 1:
            top_roller.move_relative(1000, 300);
            bottom_roller.move_relative(750, 600);
            break;
          case 2:
            bottom_roller.move_relative(750, 600);
            break;
          case 3:
            bottom_roller.move_relative(270, 600);
            break;
        }
      } else if (ball_sensor.get_value() > 2200 && ball_sensor_last) {
        ball_sensor_last = false;
      }
      if (score_queue > 0) {
        score_queue--;
        balls_in_robot--;
        top_roller.move_absolute(top_roller.get_target_position() + 500, 600);
        bottom_roller.move_absolute(bottom_roller.get_target_position() + 500, 600);
        pros::delay(500);
      }
      if (intake_queue > 0) {
        intake_left.move_velocity(200);
        intake_right.move_velocity(200);
        if (ball_sensor_triggered) {
          pros::delay(200);
          intake_queue--;
          intake_left.move_relative(-30, 200);
          intake_right.move_relative(-30, 200);
        }
      }
      pros::delay(5);
    }
  }

  void score_ball() {
    if (balls_in_robot > 0) {
      score_queue++;
    }
  }

  void test_intake() {
    intake_queue++;
  }

}

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
      intakes_retracted = true;
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
  button_handler.master.l1.pressed.set(intake_toggle);
  button_handler.master.l2.pressed.set_macro(intakes_back);
  button_handler.master.r1.pressed.set(rollers::test_intake);
  button_handler.master.r2.pressed.set(rollers::score_ball);
  button_handler.master.down.pressed.set(top_roller_reverse);
  button_handler.master.down.released.set(rollers_stop);
  button_handler.master.up.pressed.set(top_roller_forward);
  button_handler.master.up.released.set(rollers_stop);

  button_handler.partner.down.pressed.set(rollers_reverse);
  button_handler.partner.down.released.set(rollers_stop);
  button_handler.partner.up.pressed.set(rollers_forward);
  button_handler.partner.up.released.set(rollers_stop);
  button_handler.partner.left.pressed.set(remove_ball_from_robot);
  button_handler.partner.right.pressed.set(add_ball_to_robot);
  button_handler.partner.b.pressed.set(set_retracted_false);

}

} // namespace robotfunctions
