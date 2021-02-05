#ifndef ROBOT_FUNCTIONS_H
#define ROBOT_FUNCTIONS_H

#include "main.h"
#include "controller-buttons.h"
#include "utilities.h"
#include <bits/stdc++.h>

namespace robotfunctions {

class SmartMotorController {
  public:
  SmartMotorController(pros::Motor &motor, double timeout_ratio, double slew, int manual_controlls);

  pros::Motor &motor;
  double timeout_ratio;
  int auto_speed = 0;
  Slew slew;

  void run();
  void add_target(double target, int speed);
  void add_target(double target, int speed, int timeout);
  int all_manual_speeds[6];
  void set_manual_speed (int index, int speed);
  int auto_timeout(double relative_target, int speed);

  class Target {
    public:
    Target(SmartMotorController *motor_controller, double relative_target, int speed, int timeout);
    Target(SmartMotorController *motor_controller, double relative_target, int speed);
    void init_if_new();
    bool is_complete();

    SmartMotorController *motor_controller;
    double relative_target;
    int speed;
    bool is_new = true;
    int timeout;

    int time_at_start;
    double starting_pos;
  };

  std::queue<Target> target_queue = {};
};

void set_callbacks();

namespace rollers {
  extern int score_queue;
  extern int eject_queue;
  extern int intake_queue;

  extern SmartMotorController top_roller_smart;
  extern SmartMotorController bottom_roller_smart;

  void main_task();

  enum ActualColor {kRed, kBlue};
  enum AllianceColor {kOurColor, kOpposingColor};

  extern ActualColor match_color;
  extern std::deque<AllianceColor> balls_in_robot;
  extern AllianceColor last_scored_ball;

}

extern controllerbuttons::Macro intakes_back;

} // namespace robotfunctions

#endif // ROBOT_FUNCTIONS_H
