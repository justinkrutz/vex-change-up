#include "main.h"

#include "utilities.h"
#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"
#include "odom-utilities.h"
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
        if (manual_speed != 0) {
          speed = manual_speed;
          break;
        }
      }
    }
    motor.move_velocity(slew.new_value(speed) * pct_to_velocity(motor));
  }

  void SmartMotorController::add_target(double target, int speed) {
    int timeout = auto_timeout(target, speed);
    target_queue.push(Target(this, target, speed, timeout));
  }

  void SmartMotorController::add_target(double target, int speed, int timeout) {
    target_queue.push(Target(this, target, speed, timeout));
  }

  void SmartMotorController::set_manual_speed (int index, int speed) {
    all_manual_speeds[index] = speed;
  }

  int SmartMotorController::auto_timeout(double relative_target, int speed) {
    double geared_deg_per_sec;
    switch (motor.get_gearing()) {
      case pros::E_MOTOR_GEARSET_36:
        geared_deg_per_sec = DEG_PER_SEC/36;
        break;
      case pros::E_MOTOR_GEARSET_06:
        geared_deg_per_sec = DEG_PER_SEC/06;
        break;
      case pros::E_MOTOR_GEARSET_18:
      default:
        geared_deg_per_sec = DEG_PER_SEC/18;
        break;
    }
    double ms_per_deg = 1 / (geared_deg_per_sec * 0.001) * (1 / (speed * 0.01));
    return fabs(relative_target) * ms_per_deg * timeout_ratio;
  }

    SmartMotorController::Target::Target(SmartMotorController *motor_controller, double relative_target, int speed, int timeout)
        : motor_controller(motor_controller),
        relative_target(relative_target),
        speed(speed),
        timeout(timeout) {}
    // SmartMotorController::Target::Target(SmartMotorController *motor_controller, double relative_target, int speed)
    //     : motor_controller(motor_controller),
    //     relative_target(relative_target),
    //     speed(speed),
    //     timeout(auto_timeout()) {}

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


namespace rollers {
int score_queue = 0;
int eject_queue = 0;
int intake_queue = 0;
bool intake_ball = false;
bool eject_balls = false;
// bool ball_between_intake_and_rollers = false;
int time_when_last_ball_lost = 0;
double pos_when_ball_at_top = 0;
double pos_when_ball_at_bottom = 0;
SmartMotorController top_roller_smart(top_roller, 1.5, 100, 2);
SmartMotorController bottom_roller_smart(bottom_roller, 1.5, 100, 2);

const int kBallColorBlue[2] = {212, 252};
const int kBallColorRed[2] = {350, 30};
const double kBallSaturation = 0;
// const double kBallSaturation = 0.5;

enum ActualColor {kRed, kBlue};
enum AllianceColor {kOurColor, kOpposingColor};

AllianceColor get_ball_color(ActualColor match_color) {
  // optical_sensor.set_led_pwm(100);
  int hue = optical_sensor.get_hue();
  int saturation = optical_sensor.get_saturation();
  // optical_sensor.set_led_pwm(0);
  if (saturation < kBallSaturation) {
    return kOurColor;
  }

  ActualColor actual_color = match_color;

  if (hue > kBallColorBlue[0] && hue < kBallColorBlue[1]) {
    actual_color = kBlue;
  } else if (hue > kBallColorRed[0] || hue < kBallColorRed[1]) {
    actual_color = kRed;
  }
  
  if (actual_color == match_color) {
    return kOurColor;
  }
  return kOpposingColor;
}

ActualColor match_color = kRed;
std::deque<AllianceColor> balls_in_robot = {kOurColor};
AllianceColor last_scored_ball = kOurColor;

int distance_to_ball() {
  int distance_left = distance_sensor_left.get();
  int distance_right = distance_sensor_right.get();
  int min_distance = 400;
  if (distance_left != 0) {
    min_distance = distance_left;
  }
  if (distance_right != 0 && distance_right < distance_left) {
    min_distance = distance_right;
  }
  return min_distance;
}

bool score_balls = false;

void main_task() {
  intake_left.set_zero_position(360);
  intake_right.set_zero_position(360);
  intake_left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  intake_right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  bottom_roller.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  top_roller.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  ObjectSensor goal_os ({&goal_sensor_one, &goal_sensor_two}, 2800, 2850);

  // ObjectSensor ball_os_score  ({ball_sensor_score},  2000, 2200);
  ObjectSensor ball_os_score  ({&ball_sensor_top},    2000, 2200);
  ObjectSensor ball_os_top    ({&ball_sensor_top},    2200, 2400);
  ObjectSensor ball_os_middle ({&ball_sensor_middle}, 1000, 2200);
  ObjectSensor ball_os_bottom ({&ball_sensor_bottom}, 1000, 2200, true);
  ObjectSensor ball_os_intake ({&ball_sensor_intake}, 1000, 2200);

  // AllianceColor last_color = kOurColor;

  // bool ball_at_intake = false;

  bool ball_in_intake = false;
  int ball_in_intake_time = 0;
  int robot_empty_time = 0;

  pros::delay(200);
  while (true) {
    // bool ball_top_found = true;

    bool goal_os_found = goal_os.get_new_found();
    bool goal_os_lost = goal_os.get_new_lost();


    bool ball_top_found = ball_os_top.get_new_found();
    bool ball_top_lost = ball_os_top.get_new_lost(fabs(pos_when_ball_at_top - top_roller.get_position()) > 50);

    bool ball_score_found = ball_os_score.get_new_found();
    bool ball_score_lost = ball_os_score.get_new_lost(top_roller.get_position() - pos_when_ball_at_top  > 90);
    
    bool ball_middle_found = ball_os_middle.get_new_found();
    bool ball_middle_lost = ball_os_middle.get_new_lost(pos_when_ball_at_top - top_roller.get_position() > 50
        || pos_when_ball_at_bottom - bottom_roller.get_position() < -50);
    
    bool ball_bottom_found = ball_os_bottom.get_new_found();
    bool ball_bottom_lost = ball_os_bottom.get_new_lost(fabs(pos_when_ball_at_bottom - bottom_roller.get_position()) > 50);

    bool ball_intake_found = ball_os_intake.get_new_found();
    bool ball_intake_lost = ball_os_intake.get_new_lost();

    std::vector<ObjectSensor *> object_sensors = {&ball_os_top, &ball_os_middle, &ball_os_bottom};
    int balls_definitely_in_robot = 0;
    int balls_possibly_in_robot = 3;
    for (auto &&object_sensor : object_sensors) {
      if (object_sensor->is_detected && pros::millis() - object_sensor->time_when_found > 2000) {
        balls_definitely_in_robot++;
      } else if (!object_sensor->is_detected && pros::millis() - object_sensor->time_when_lost > 2000) {
        balls_possibly_in_robot--;
      }
    }
    int balls_to_push = balls_definitely_in_robot - balls_in_robot.size();
    int balls_to_pop = balls_in_robot.size() - balls_possibly_in_robot;
    for (int i = 0; i < balls_to_push; i++) {
      balls_in_robot.push_back(kOurColor);
    }
    for (int i = 0; i < balls_to_pop; i++) {
      balls_in_robot.pop_front();
    }
    
    
    

    if (ball_intake_found) {
      // bottom_roller_smart.set_manual_speed(4, 100);
      bottom_roller_smart.add_target(1200, 100);
    } else if (ball_intake_lost && bottom_roller.get_actual_velocity() < -10) {
      balls_in_robot.pop_back(); // remove the bottom ball from the robot because it was ejected
    }

    if (ball_bottom_found && bottom_roller.get_actual_velocity() > 10) {
        if (balls_in_robot.size() >= 3) balls_in_robot.pop_front(); // balls_in_robot.size() was 3 but the top ball was removed because a fourth cannot be added
        balls_in_robot.push_back(get_ball_color(match_color)); // add ball to the robot

        switch (balls_in_robot.size()) {
          case 1:
            top_roller_smart.set_manual_speed(3, 100);
            break;
          // case 3:
          //   bottom_roller_smart.set_manual_speed(4, 0); // stop because the robot is full
          //   break;
        }
    } else if (ball_bottom_lost) {
      ball_in_intake = false;
    }
    
    if (ball_middle_found && bottom_roller.get_actual_velocity() > 10) {
        if (balls_in_robot.size() > 1) bottom_roller_smart.set_manual_speed(4, 0);
    }

    if (ball_top_found) {
      if (top_roller.get_actual_velocity() < -10) {
        if (balls_in_robot.size() >= 3) balls_in_robot.pop_back();
        balls_in_robot.push_front(get_ball_color(match_color));
      } else {
        top_roller_smart.set_manual_speed(3, 0);
        bottom_roller_smart.set_manual_speed(3, 0);
        pos_when_ball_at_top = top_roller.get_position();
      }
    } else if (ball_top_lost) {
      if (balls_in_robot.size() > 0) {
        last_scored_ball = balls_in_robot.back();
        balls_in_robot.pop_front();
      }
      if (balls_in_robot.size() > 0) {
        top_roller_smart.set_manual_speed(3, 100);
        bottom_roller_smart.set_manual_speed(3, 50);
      } else {
        robot_empty_time = pros::millis();
      }
    }

    if (ball_score_lost || (balls_in_robot.size() == 0 && pros::millis() - robot_empty_time > 500)) {
      if (score_queue > 0) score_queue--;
    }

    if (score_queue > 0 && (score_balls || goal_os.is_detected)) {
      score_balls = false;
      top_roller_smart.set_manual_speed(2, 100);
      bottom_roller_smart.set_manual_speed(2, 100);
    } else if (score_queue == 0) {
      top_roller_smart.set_manual_speed(2, 0);
      bottom_roller_smart.set_manual_speed(2, 0);
    }

    if (intake_ball && !ball_in_intake && intake_queue == 0 && InRange(distance_to_ball(), 50, 300)) {
      intake_queue = 1;
      ball_in_intake = true;
      ball_in_intake_time = pros::millis();
    }

    if (ball_in_intake && pros::millis() - ball_in_intake_time > 500 && !InRange(distance_to_ball(), 40, 200)) {
      intake_splay();
      ball_in_intake = false;
    }

    if (intake_queue > 0) {
      if (ball_bottom_found) {
        if (intake_queue == 1) {
          intake_splay();
        } else {
          intake_queue--;
        }
      } else {
        intake_left.move_velocity(200);
        intake_right.move_velocity(200);
      }
    }

    top_roller_smart.run();
    bottom_roller_smart.run();

    std::string ball_string = "";
    for (int i = 0; i < balls_in_robot.size(); i++)
    {
      ball_string += std::to_string(balls_in_robot[i]) + " ";
    }
    // controllermenu::master_print_array[0] = ball_string;
    // controllermenu::partner_print_array[0] = "SQ: " + std::to_string(score_queue) + " IQ: " + std::to_string(intake_queue);
    // controllermenu::partner_print_array[1] = "GS1: " + std::to_string(goal_sensor_one.get_value()) + " GS2: " + std::to_string(goal_sensor_two.get_value());
    // controllermenu::partner_print_array[2] = "MIN: " + std::to_string(goal_os.get_min_value()) + " MAX: " + std::to_string(goal_os.get_max_value());
    pros::delay(10);
  }
}

void score_balls_true() {
  score_balls = true;
}

void score_balls_false() {
  score_balls = false;
}

// void eject_balls() {
//   eject_balls = true;
// }

void add_ball_to_intake_queue() {
  intake_queue++;
}

void intake_ball_true() {
  intake_ball = true;
}

void intake_ball_false() {
  intake_ball = false;
}

void intake_deploy_off() {
  intake_left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake_right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake_left.move_relative(180, 200);
  intake_right.move_relative(180, 200);
}

void intake_off() {
  intake_left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake_right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake_left = 0;
  intake_right = 0;
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
  // button_handler.master.r2.pressed.set(intake_ball_true);
  // button_handler.master.r2.released.set(intake_ball_false);
  button_handler.master. r2.   pressed .set([&](){ eject_balls = true; });
  button_handler.master. r2.   released.set([&](){ eject_balls = false; });
  button_handler.master. r1.   pressed .set(add_ball_to_intake_queue);
  button_handler.master. down. pressed .set(rollers_reverse);
  button_handler.master. down. released.set(rollers_stop);
  button_handler.master. up.   pressed .set(rollers_forward);
  button_handler.master. up.   released.set(rollers_stop);
  button_handler.master. x.    pressed .set(bottom_roller_forward);
  button_handler.master. x.    released.set(rollers_stop);
  button_handler.master. b.    pressed .set(bottom_roller_reverse);
  button_handler.master. b.    released.set(rollers_stop);
  button_handler.master. a.    pressed .set(intake_deploy_off);
  button_handler.master. y.    pressed .set(intake_off);

  button_handler.partner.l2.   pressed .set(rollers_reverse);
  button_handler.partner.l2.   released.set(rollers_stop);
  button_handler.partner.l1.   pressed .set(rollers_forward);
  button_handler.partner.l1.   released.set(rollers_stop);

  button_handler.partner.r2.   pressed .set(score_balls_true);
  button_handler.partner.r2.   released.set(score_balls_false);
  button_handler.partner.up.   pressed .set([&](){ score_queue = 1; });
  button_handler.partner.right.pressed .set([&](){ score_queue = 2; });
  button_handler.partner.down. pressed .set([&](){ score_queue = 3; });
  button_handler.partner.left. pressed .set([&](){ score_queue = 0; });
  button_handler.partner.x.    pressed .set([&](){ intake_queue = 1; });
  button_handler.partner.a.    pressed .set([&](){ intake_queue = 2; });
  button_handler.partner.b.    pressed .set([&](){ intake_queue = 3; });
  button_handler.partner.y.    pressed .set(intake_splay);
}

} // namespace robotfunctions
