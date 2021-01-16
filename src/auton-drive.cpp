#include "main.h"

#include "utilities.h"
#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"
#include "auton-drive.h"
#include "auton-from-sd.h"
#include <stdio.h>
#include <complex.h>

namespace autondrive {

controllerbuttons::MacroGroup auton_group;

OdomState tracking_to_robot_coords (OdomState tracking_coords) {
  // QLength x_offset = cos(tracking_coords.theta.convert(radian)) * TRACKING_ORIGIN_OFFSET;
  // QLength y_offset = sin(tracking_coords.theta.convert(radian)) * TRACKING_ORIGIN_OFFSET;

  // return {tracking_coords.x + x_offset, tracking_coords.y + y_offset, tracking_coords.theta};
  return tracking_coords;
}

OdomState robot_to_tracking_coords (OdomState robot_coords) {
  // QLength x_offset = cos(robot_coords.theta.convert(radian)) * -TRACKING_ORIGIN_OFFSET;
  // QLength y_offset = sin(robot_coords.theta.convert(radian)) * -TRACKING_ORIGIN_OFFSET;

  // return {robot_coords.x + x_offset, robot_coords.y + y_offset, robot_coords.theta};
  return robot_coords;
}

OdomState robot_state() {
  return tracking_to_robot_coords(chassis->getState());
}

Target::Target(QLength x, QLength y, QAngle theta) : x(x), y(y), theta(theta) {}

void Target::init_if_new() {
  if (is_new) {
    is_new = false;
    starting_state = robot_state();
  }
}

namespace drivetoposition {
std::queue<Target> targets;
bool target_position_enabled = false;
bool final_target_reached = true;
bool target_heading_reached = false;
bool target_distance_reached = false;

OdomState starting_position;
// RampMathSettings move_settings = {20, 100, 20, 0.5, 0.5};
// RampMathSettings turn_settings = {20, 100, 20, 0.1, 0.1};
RampMathSettings move_settings = {20, 100, 15, 0.1, 0.2};
RampMathSettings turn_settings = {10, 50, 10, 0.1, 0.1};

double forward = 0;
double strafe  = 0;
double turn    = 0;

void add_position_target(QLength x, QLength y, QAngle theta, QLength offset) {
  QLength x_offset = cos(theta) * offset;
  QLength y_offset = sin(theta) * offset;
  targets.push({x - x_offset, y - y_offset, theta});
  target_position_enabled = true;
  final_target_reached = false;
};


void update() {
  if (target_position_enabled && targets.size() > 0) {
    Target &target = targets.front();
    target.init_if_new();

    double move_speed;
    double turn_speed;

    Point target_point{target.x, target.y};

    OdomState target_state{target.x, target.y, target.theta};
    Point starting_point{target.starting_state.x, target.starting_state.y};

    auto [distance_to_target, direction] = OdomMath::computeDistanceAndAngleToPoint(target_point, robot_state());
    QLength distance_traveled = OdomMath::computeDistanceToPoint(starting_point, robot_state());
    QLength total_distance = OdomMath::computeDistanceToPoint(starting_point, target_state);

    QAngle total_angle = target.theta - target.starting_state.theta;


    if (distance_traveled >= total_distance) {
      target_distance_reached = true;
    }

    if (fabs(robot_state().theta.convert(degree) - target_state.theta.convert(degree)) < 2.5) {
      target_heading_reached = true;
    }


    if (target_heading_reached && target_distance_reached) {
      if (targets.size() > 1) {
        target_heading_reached = false;
        target_distance_reached = false;
        targets.pop();
        return;
      } else {
        final_target_reached = true;
      }
    } 
    
    if (target_distance_reached) {
      move_speed = std::min(100.0, 5 * distance_to_target.convert(inch));
    } else {
      move_speed = rampMath(distance_traveled.convert(inch), total_distance.convert(inch), move_settings);
    }

    if (target_heading_reached) {
      turn_speed = std::min(100.0, 100 * (target.theta - robot_state().theta).convert(radian));
    } else {
      turn_speed = std::min(100.0, 100 * (target.theta - robot_state().theta).convert(radian));
    }

    forward = move_speed * cos(direction.convert(radian));
    strafe  = move_speed * sin(direction.convert(radian));
    turn    = turn_speed;

  } else {
    forward = 0;
    strafe = 0;
    turn = 0;
    targets = {};
  }
}
}

double button_strafe = 0;
double button_turn = 0;
double button_forward = 0;

void motor_task()
{
  std::shared_ptr<AbstractMotor> drive_fl = x_model->getTopLeftMotor();
  std::shared_ptr<AbstractMotor> drive_fr = x_model->getTopRightMotor();
  std::shared_ptr<AbstractMotor> drive_bl = x_model->getBottomLeftMotor();
  std::shared_ptr<AbstractMotor> drive_br = x_model->getBottomRightMotor();

  // int time_last = 0;
  Slew drive_fl_slew(DRIVER_SLEW);
  Slew drive_fr_slew(DRIVER_SLEW);
  Slew drive_bl_slew(DRIVER_SLEW);
  Slew drive_br_slew(DRIVER_SLEW);

  while(1)
  {
    drivetoposition::update();

    double forward = button_forward + drivetoposition::forward + master.get_analog(ANALOG_RIGHT_Y) * 0.787401574803;
    // double strafe  = button_strafe + drivetoposition::strafe;
    double strafe  = button_strafe + drivetoposition::strafe  + master.get_analog(ANALOG_RIGHT_X) * 0.787401574803;
    double temp_turn  = master.get_analog(ANALOG_LEFT_X) * 0.787401574803;
    double turn    = button_turn + drivetoposition::turn    + pow(abs(temp_turn / 100), 1.8) * 100 * sgn(temp_turn);
    double sync = std::min(1.0, 100 / (fabs(forward) + fabs(strafe) + fabs(turn)));

    double drive_fl_pct = drive_fl_slew.new_value((forward + strafe + turn) * sync);
    double drive_fr_pct = drive_fr_slew.new_value((forward - strafe - turn) * sync);
    double drive_bl_pct = drive_bl_slew.new_value((forward - strafe + turn) * sync);
    double drive_br_pct = drive_br_slew.new_value((forward + strafe - turn) * sync);

    drive_fl->moveVelocity(drive_fl_pct * 2);
    drive_fr->moveVelocity(drive_fr_pct * 2);
    drive_bl->moveVelocity(drive_bl_pct * 2);
    drive_br->moveVelocity(drive_br_pct * 2);

    pros::delay(5);
  }
}

void goal_turn_right() {
  button_strafe = 100;
  button_turn = -68.4;
  button_forward = 10;
}

void goal_turn_left() {
  button_strafe = -100;
  button_turn = 80;
  button_forward = 10;
}

void goal_turn_release() {
  button_strafe = 0;
  button_turn = 0;
  button_forward = 0;
}

void set_callbacks() {
  using namespace controllerbuttons;
  button_handler.master.left.pressed.set(goal_turn_left);
  button_handler.master.right.pressed.set(goal_turn_right);
  button_handler.master.left.released.set(goal_turn_release);
  button_handler.master.right.released.set(goal_turn_release);
}

} // namespace autondrive

namespace autonroutines {
using namespace autondrive;
using namespace drivetoposition;
using namespace controllerbuttons;
using namespace robotfunctions;
using namespace rollers;

Macro none([&](){},[](){});

Macro test(
    [&](){
      chassis->setState(robot_to_tracking_coords({0_in, 0_in, 0_deg}));
      // add_position_target(15_in, 15_in, -90_deg);
      // wait(3000);
      // add_position_target(15_in, 15_in, 0_deg);
      // wait(3000);
      // add_position_target(15_in, 0_in, 0_deg);
      // wait(3000);
      add_position_target(0_in, 0_in, 0_deg);
      wait(500);
      add_position_target(0_in, 0_in, 360_deg);
      wait(500);
      add_position_target(0_in, 0_in, 0_deg);
      wait(3000);
      // WAIT_UNTIL(final_target_reached);
    },
    [](){
      target_position_enabled = false;
    },
    {&auton_group});

Macro home_row_three(
    [&](){
      chassis->setState(robot_to_tracking_coords({15.7416_in, 31.4911_in, -90_deg}));

      move_settings.start_output = 100;
      move_settings.end_output = 20;
      
      add_position_target(26.319_in, 26.319_in, -90_deg);
      add_position_target(26.319_in, 26.319_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 10;
      wait(400);
      top_roller_smart.add_target(45, 30);
      add_position_target(20.4_in, 20.4_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      wait(400);
      intake_queue = 0;
      intakes_back.start();
      wait(300);
      add_position_target(16_in, 16_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      add_position_target(13.6_in, 13.6_in, -135_deg);
      wait(200);
      score_queue = 1;
      wait(200);
      targets.pop();
      // add_position_target(30_in, 30_in, -135_deg);
      add_position_target(26_in, 72_in, -180_deg);
      // add_position_target(22_in, 72_in, -180_deg);

      WAIT_UNTIL(final_target_reached)
      add_position_target(17_in, 72_in, -180_deg);
      move_settings.start_output = 20;
      move_settings.end_output = 20;
      wait(500);
      score_queue = 1;
      wait(200);
      targets.pop();
      wait(20);
      // chassis->setState(robot_to_tracking_coords({20.75_in, 71.63_in, -171.5_deg}));

      move_settings.start_output = 100;
      move_settings.end_output = 50;
      add_position_target(32_in, 72_in, -180_deg);
      WAIT_UNTIL(final_target_reached)
      move_settings.end_output = 20;

      // add_position_target(32_in, 109_in, -180_deg);
      add_position_target(32_in, 116_in, -180_deg);
      // wait(500);
      // add_position_target(32_in, 109_in, -225_deg);
      add_position_target(32_in, 116_in, -225_deg);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(500);
      add_position_target(23_in, 126_in, -225_deg);
      WAIT_UNTIL(final_target_reached)
      wait(300);
      intake_queue = 0;
      intakes_back.start();
      wait(30);
      intakes_back.terminate();
      wait(270);
      // add_position_target(13.6_in, 127_in, -225_deg);
      add_position_target(6_in, 150.85_in, -225_deg);
      wait(500);
      score_queue = 1;
      wait(200);
      targets.pop();
      wait(20);
      // wait(200);
      // add_position_target(32_in, 109_in, -180_deg);
      add_position_target(32_in, 118_in, -225_deg);
      WAIT_UNTIL(final_target_reached)
      score_queue = 0;
    },
    [](){
      target_position_enabled = false;
    },
    {&auton_group});

Macro home_row_two(
    [&](){
      chassis->setState(robot_to_tracking_coords({15.7416_in, 31.4911_in, -90_deg}));

      move_settings.start_output = 100;
      move_settings.end_output = 20;
      
      add_position_target(26.319_in, 26.319_in, -90_deg);
      add_position_target(26.319_in, 26.319_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 10;
      wait(400);
      top_roller_smart.add_target(45, 30);
      add_position_target(20.4_in, 20.4_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      wait(400);
      intake_queue = 0;
      intakes_back.start();
      wait(300);
      add_position_target(16_in, 16_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      add_position_target(13.6_in, 13.6_in, -135_deg);
      wait(200);
      score_queue = 1;
      wait(200);
      targets.pop();
      // add_position_target(30_in, 30_in, -135_deg);
      add_position_target(26_in, 72_in, -180_deg);
      // add_position_target(22_in, 72_in, -180_deg);

      WAIT_UNTIL(final_target_reached)
      add_position_target(17_in, 72_in, -180_deg);
      move_settings.start_output = 20;
      move_settings.end_output = 20;
      wait(500);
      score_queue = 1;
      wait(200);
      targets.pop();
      wait(20);
      move_settings.start_output = 100;
      move_settings.end_output = 50;
      add_position_target(32_in, 72_in, -180_deg);
      WAIT_UNTIL(final_target_reached)
      move_settings.end_output = 20;
      score_queue = 0;
    },
    [](){
      target_position_enabled = false;
    },
    {&auton_group});

Macro left_shawnton(
    [&](){
      chassis->setState(robot_to_tracking_coords({15.7416_in, 31.4911_in, -90_deg}));

      add_position_target(26.319_in, 26.319_in, -90_deg);
      add_position_target(26.319_in, 26.319_in, -135_deg);
      add_position_target(16_in, 16_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      add_position_target(13.6_in, 13.6_in, -135_deg);
      intake_left.move_relative(30, 200);
      intake_right.move_relative(30, 200);
      wait(500);
      intake_left.move_relative(-30, 200);
      intake_right.move_relative(-30, 200);
      top_roller_smart.add_target(45, 30);
      wait(500);
      score_queue = 1;
      wait(200);
      targets.pop();
      wait(500);
      add_position_target(30_in, 30_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
    },
    [](){
      target_position_enabled = false;
    },
    {&auton_group});

Macro right_shawnton(
    [&](){
      chassis->setState(robot_to_tracking_coords({31.4911_in, 15.7416_in, -180_deg}));

      add_position_target(26.319_in, 26.319_in, -180_deg);
      WAIT_UNTIL(final_target_reached)
      add_position_target(26.319_in, 26.319_in, -135_deg);
      wait(500);
      targets.pop();
      add_position_target(16_in, 16_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      add_position_target(13.6_in, 13.6_in, -135_deg);
      intake_left.move_relative(30, 200);
      intake_right.move_relative(30, 200);
      wait(500);
      intake_left.move_relative(-30, 200);
      intake_right.move_relative(-30, 200);
      top_roller_smart.add_target(45, 30);
      wait(500);
      score_queue = 1;
      wait(200);
      targets.pop();
      wait(500);
      add_position_target(30_in, 30_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
    },
    [](){
      target_position_enabled = false;
    },
    {&auton_group});

Macro skills(
    [&](){
      chassis->setState(robot_to_tracking_coords({13.491_in, 34.9911_in, 0_deg}));

      move_settings.start_output = 100;
      move_settings.end_output = 20;

      intake_queue = 1;
      add_position_target(18_in, 34.9911_in, 0_deg);
      add_position_target(5.8129_in, 5.8129_in, -135_deg, 17_in);
      wait(500);
      intakes_back.start();
      WAIT_UNTIL(final_target_reached)
      add_position_target(5.8129_in, 5.8129_in, -135_deg, 6_in);
      wait(300);
      score_queue = 1;
      wait(300);
      targets.pop();

      score_queue = 0;
      add_position_target(5.9272_in, 70.3361_in, -180_deg, 20_in);
      WAIT_UNTIL(final_target_reached)
      add_position_target(5.9272_in, 70.3361_in, -180_deg, 6_in);
      wait(600);
      score_queue = 1;
      wait(400);
      targets.pop();

      score_queue = 0;
      // move_settings.end_output = 100;
      add_position_target(23_in, 70.3361_in, -180_deg);
      add_position_target(23_in, 90_in, -270_deg);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      add_position_target(23_in, 117.18_in, -270_deg);
      // move_settings.end_output = 20;
      WAIT_UNTIL(final_target_reached)
      intakes_back.start();
      add_position_target(5.8129_in, 134.8593_in, -225_deg, 24.5_in);
      WAIT_UNTIL(final_target_reached)
      add_position_target(5.8129_in, 134.8593_in, -225_deg, 6_in);
      wait(700);
      score_queue = 1;
      wait(400);
      targets.pop();

      score_queue = 0;
      add_position_target(23_in, 117.18_in, -270_deg);
      WAIT_UNTIL(final_target_reached)
      intake_right.move_relative(180, 200);
      add_position_target(34.8361_in, 125.6722_in, -270_deg);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(1000);
      intakes_back.start();
      add_position_target(70.3361_in, 117.4624_in, -360_deg, 12_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(1000);
      add_position_target(70.3361_in, 117.4624_in, -360_deg);
      intakes_back.start();
      add_position_target(70.3361_in, 117.4624_in, -270_deg);
      WAIT_UNTIL(final_target_reached)
      add_position_target(70.3361_in, 134.745_in, -270_deg, 6_in);
      wait(500);
      score_queue = 2;
      wait(700);
      targets.pop();

      score_queue = 0;
      add_position_target(70.3361_in, 117.4624_in, -270_deg);
      WAIT_UNTIL(final_target_reached)
      intake_right.move_relative(180, 200);
      add_position_target(105.8361_in, 125.6722_in, -270_deg);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(1000);
      intakes_back.start();
      add_position_target(134.8593_in, 134.8593_in, -315_deg, 17_in);
      WAIT_UNTIL(final_target_reached)
      add_position_target(134.8593_in, 134.8593_in, -315_deg, 6_in);
      wait(300);
      score_queue = 1;
      wait(300);
      targets.pop();

      score_queue = 0;
      add_position_target(118_in, 125_in, -360_deg);
      add_position_target(118_in, 125_in, -450_deg);
      add_position_target(117.6361_in, 105.6361_in, -450_deg, 10_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      add_position_target(117.6361_in, 70.3361_in, -450_deg);
      wait(1000);
      intakes_back.start();
      add_position_target(117.6361_in, 70.3361_in, -360_deg);
      WAIT_UNTIL(final_target_reached)
      add_position_target(134.745_in,  70.3361_in, -360_deg, 6_in);
      wait(1000);
      score_queue = 1;
      wait(200);
      targets.pop();

      score_queue = 0;
      add_position_target(117.6361_in, 70.3361_in, -360_deg);
      add_position_target(117.6361_in, 35.0361_in, -450_deg, 10_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      add_position_target(117.1816_in, 23.4906_in, -450_deg);
      WAIT_UNTIL(final_target_reached)
      intakes_back.start();
      add_position_target(134.8593_in, 5.8129_in, -405_deg, 24.5_in);
      WAIT_UNTIL(final_target_reached)
      add_position_target(134.8593_in, 5.8129_in, -405_deg, 6_in);
      wait(700);
      score_queue = 1;
      wait(400);
      targets.pop();

      score_queue = 0;
      add_position_target(117.1816_in, 23.4906_in, -450_deg);
      WAIT_UNTIL(final_target_reached)
      intake_right.move_relative(180, 200);
      add_position_target(105.8361_in, 3.3361_in, -450_deg , 13_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(1000);
      intakes_back.start();
      add_position_target(70.3361_in, 23.3361_in, -540_deg, 11_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(1000);
      add_position_target(70.3361_in, 23.3361_in, -540_deg);
      intakes_back.start();
      add_position_target(70.3361_in, 23.3361_in, -450_deg);
      WAIT_UNTIL(final_target_reached)
      add_position_target(70.3361_in, 5.9272_in, -450_deg, 6_in);
      wait(600);
      score_queue = 2;
      wait(700);
      targets.pop();

      score_queue = 0;
      add_position_target(70.3361_in, 23.3361_in, -450_deg);
      add_position_target(70.3361_in, 23.3361_in, -270_deg);
      add_position_target(70.3361_in, 46.8361_in, -270_deg, 10_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(1000);
      add_position_target(70.3361_in, 70.3361_in, -270_deg, 16_in);
      WAIT_UNTIL(final_target_reached)
      add_position_target(70.3361_in, 70.3361_in, -270_deg, 6_in);
      wait(300);
      intake_queue = 50;
      wait(200);
      targets.pop();

      button_strafe = 50;
      button_turn = -34.2;
      button_forward = 10;
      wait(2000);
      button_strafe = -50;
      button_turn = 40;
      button_forward = 10;
      wait(4000);
      goal_turn_release();

      intakes_back.start();
      score_queue = 1;
    },
    [](){
      goal_turn_release();
      target_position_enabled = false;
    },
    {&auton_group});

} // namespace autonroutines