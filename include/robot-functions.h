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
  void add_target(double target, int speed, double timeout);
  int all_manual_speeds[2];
  void set_manual_speed (int index, int speed);

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
    int auto_timeout();
    int timeout;

    int time_at_start;
    double starting_pos;
  };

  std::queue<Target> target_queue = {};
};

struct {
  double forward;
  double strafe;
  double turn;
} set_drive;

// void driveToPosition(QLength x, QLength y, QAngle theta, QLength offset = 0_in);
void intakeBalls(int balls);
void scoreBalls(int balls);

void count_up_task();
void count_down_task();
void count_task(void * arg);
void single_use_button();

void check_for_warnings();

void set_callbacks();
// void motor_task();

namespace rollers {
  extern int score_queue;
  extern int intake_queue;
  
  extern SmartMotorController top_roller_smart;
  extern SmartMotorController bottom_roller_smart;

  void main_task();
  void score_ball();
  void test_intake();
}

extern controllerbuttons::Macro intakes_back;

} // namespace robotfunctions

#endif // ROBOT_FUNCTIONS_H
