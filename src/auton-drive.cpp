#include "main.h"

#include "utilities.h"
#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"
#include "auton-drive.h"
#include "odom-utilities.h"
#include <stdio.h>
#include <complex.h>

namespace autondrive {

controllerbuttons::MacroGroup auton_group;
controllerbuttons::MacroGroup drive_group;

OdomState get_odom_state() {
  return chassis->getState();
}

double button_strafe = 0;
double button_turn = 0;
double button_forward = 0;

namespace drivetoposition {

Target::Target(QLength x, QLength y, QAngle theta) : x(x), y(y), theta(theta) {}

void Target::init_if_new() {
  if (is_new) {
    is_new = false;
    starting_state = get_odom_state();
  }
}

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

void add_target(QLength x, QLength y, QAngle theta, QLength offset_distance, QAngle offset_angle) {
  QLength x_offset = cos(theta) * offset_distance;
  QLength y_offset = sin(theta) * offset_distance;
  targets.push({x - x_offset, y - y_offset, theta});
  target_position_enabled = true;
  final_target_reached = false;
}

void add_target(QLength x, QLength y, QAngle theta, QLength offset_distance) {
  add_target(x, y, theta, offset_distance, theta);
}

void add_target(QLength x, QLength y, QAngle theta) {
  add_target(x, y, theta, 0_in);
}

void add_target(odomutilities::Goal goal, QAngle theta, QLength offset_distance, QAngle offset_angle) {
  add_target(goal.point.x, goal.point.y, theta, offset_distance, offset_angle);
}

void add_target(odomutilities::Goal goal, QAngle theta, QLength offset_distance) {
  add_target(goal, theta, offset_distance, theta);
}

void add_target(odomutilities::Goal goal, QAngle theta) {
  add_target(goal, theta, 0_in);
}

void wait_until_final_target_reached() {
  while (!final_target_reached) {
    controllerbuttons::wait(10);
  };
}

void clear_targets() {
  targets = {};
}

using namespace controllerbuttons;

void drive_to_goal(odomutilities::Goal goal, QAngle angle) {
  ObjectSensor goal_os ({&goal_sensor_one, &goal_sensor_two}, 2800, 2850);
  add_target(goal.point.y, goal.point.x, angle, 12.4_in);
  while (!goal_os.get_new_found()) {
    if (final_target_reached) {
      button_forward = 0.2 * goal_os.get_min_value() - 2300;
    }
    wait(10);
  }
  targets = {};
  button_forward = 0;
}

void drive_to_goal(odomutilities::Goal goal) {
  drive_to_goal(goal, goal.angles[0]);
}


void update() {
  if (target_position_enabled && targets.size() > 0) {
    Target &target = targets.front();
    target.init_if_new();

    double move_speed;
    double turn_speed;

    Point target_point{target.x, target.y};

    OdomState target_state{target.x, target.y, target.theta};
    Point starting_point{target.starting_state.x, target.starting_state.y};

    auto [distance_to_target, direction] = OdomMath::computeDistanceAndAngleToPoint(target_point, get_odom_state());
    QLength distance_traveled = OdomMath::computeDistanceToPoint(starting_point, get_odom_state());
    QLength total_distance = OdomMath::computeDistanceToPoint(starting_point, target_state);

    QAngle total_angle = target.theta - target.starting_state.theta;


    if (distance_traveled >= total_distance) {
      target_distance_reached = true;
    }

    if (abs(get_odom_state().theta - target_state.theta) < 2.5_deg) {
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
      turn_speed = std::min(100.0, 100 * (target.theta - get_odom_state().theta).convert(radian));
    } else {
      turn_speed = std::min(100.0, 100 * (target.theta - get_odom_state().theta).convert(radian));
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

} // namespace drivetoposition


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
    double strafe  = button_strafe + drivetoposition::strafe  + master.get_analog(ANALOG_LEFT_X) * 0.787401574803;
    // double strafe  = button_strafe + drivetoposition::strafe  + master.get_analog(ANALOG_RIGHT_X) * 0.787401574803;
    double temp_turn  = master.get_analog(ANALOG_RIGHT_X) * 0.787401574803;
    double turn    = button_turn + drivetoposition::turn + temp_turn;
    // double turn    = button_turn + drivetoposition::turn    + pow(abs(temp_turn / 100), 1.8) * 100 * sgn(temp_turn);
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



using namespace controllerbuttons;



Macro goal_center(
    [&](){
      using namespace odomutilities;
      int time = pros::millis();
      OdomState odom = get_odom_state();
      Goal *closest_goal = Goal::closest({odom.x, odom.y});
      QAngle target = 0_deg;

      for (auto &&angle : closest_goal->angles) {
        QAngle target_right = (odom.theta.convert(degree) - fmod(odom.theta.convert(degree), 360)) * degree + angle;
        QAngle target_left = target_right - 360_deg;
        QAngle temp_target;

        if (abs(target_right - odom.theta) < abs(target_left - odom.theta)) {
          temp_target = target_right;
        } else {
          temp_target = target_left;
        }

        if (abs(temp_target - odom.theta) < abs(target - odom.theta)) {
          target = temp_target;
        }
      }
      

      while (abs(get_odom_state().theta - target) > 1_deg && pros::millis() - time < 2000) {
        double speed = 2 * (target - get_odom_state().theta).convert(degree);
        // controllermenu::partner_print_array[0] = "T " + std::to_string(target.convert(degree));
        // controllermenu::partner_print_array[1] = "S " + std::to_string(speed);
        button_strafe = -speed;
        button_turn = speed * 0.7;
        button_forward = 0.04 * (MIN(goal_sensor_one.get_value(), goal_sensor_two.get_value() - 2300));
        wait(10);
      }
    },
    [](){
        button_strafe = 0;
        button_turn = 0;
        button_forward = 0;
    },
    {&drive_group});

Macro goal_turn_right(
    [&](){
      button_strafe = 100;
      button_turn = -70;
      while (true) {
        button_forward = 0.04 * (MIN(goal_sensor_one.get_value(), goal_sensor_two.get_value() - 2300));
        wait(10);
      }
    },
    [](){
        button_strafe = 0;
        button_turn = 0;
        button_forward = 0;
    },
    {&drive_group});

Macro goal_turn_left(
    [&](){
      button_strafe = -100;
      button_turn = 80;
      while (true) {
        button_forward = 0.04 * (MIN(goal_sensor_one.get_value(), goal_sensor_two.get_value() - 2300));
        wait(10);
      }
    },
    [](){
        button_strafe = 0;
        button_turn = 0;
        button_forward = 0;
    },
    {&drive_group});

void drive_group_terminate() {
  drive_group.terminate();
}

void set_callbacks() {
  using namespace controllerbuttons;
  button_handler.master.left.pressed.set_macro(goal_turn_left);
  button_handler.master.right.pressed.set_macro(goal_turn_right);
  button_handler.master.left.released.set(drive_group_terminate);
  button_handler.master.right.released.set(drive_group_terminate);
}

} // namespace autondrive

namespace autonroutines {
using namespace autondrive;
using namespace drivetoposition;
using namespace odomutilities;
using namespace controllerbuttons;
using namespace robotfunctions;
using namespace rollers;

Macro none([&](){},[](){});

Macro test(
    [&](){
      chassis->setState({0_in, 0_in, 0_deg});
      // add_target(15_in, 15_in, -90_deg);
      // wait(3000);
      // add_target(15_in, 15_in, 0_deg);
      // wait(3000);
      // add_target(15_in, 0_in, 0_deg);
      // wait(3000);
      add_target(0_in, 0_in, 0_deg);
      wait(500);
      add_target(0_in, 0_in, 360_deg);
      wait(500);
      add_target(0_in, 0_in, 0_deg);
      wait(3000);
      // WAIT_UNTIL(final_target_reached);
    },
    [](){
      target_position_enabled = false;
    },
    {&auton_group});

Macro home_row_three(
    [&](){
      chassis->setState({15.7416_in, 31.4911_in, -90_deg});

      move_settings.start_output = 100;
      move_settings.end_output = 20;
      
      add_target(26.319_in, 26.319_in, -90_deg);
      add_target(26.319_in, 26.319_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 10;
      wait(400);
      top_roller_smart.add_target(45, 30);
      add_target(20.4_in, 20.4_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      wait(400);
      intake_queue = 0;
      intakes_back.start();
      wait(300);
      add_target(16_in, 16_in, -135_deg);
      wait(10);
      WAIT_UNTIL(final_target_reached)
      add_target(13.6_in, 13.6_in, -135_deg);
      wait(200);
      score_queue = 1;
      wait(200);
      targets.pop();
      // add_target(30_in, 30_in, -135_deg);
      add_target(26_in, 72_in, -180_deg);
      // add_target(22_in, 72_in, -180_deg);

      wait(10);
      WAIT_UNTIL(final_target_reached)
      add_target(17_in, 72_in, -180_deg);
      move_settings.start_output = 20;
      move_settings.end_output = 20;
      wait(700
      );
      score_queue = 1;
      wait(200);
      targets.pop();
      wait(10);
      // chassis->setState({20.75_in, 71.63_in, -171.5_deg});

      move_settings.start_output = 100;
      move_settings.end_output = 50;
      wait(10);
      add_target(32_in, 72_in, -180_deg);
      wait(10);
      WAIT_UNTIL(final_target_reached)
      move_settings.end_output = 20;

      // add_target(32_in, 109_in, -180_deg);
      add_target(32_in, 116_in, -180_deg);
      // wait(500);
      // add_target(32_in, 109_in, -225_deg);
      add_target(32_in, 116_in, -225_deg);
      wait(10);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(500);
      add_target(23_in, 126_in, -225_deg);
      wait(10);
      WAIT_UNTIL(final_target_reached)
      wait(300);
      intake_queue = 0;
      intakes_back.start();
      wait(30);
      intakes_back.terminate();
      wait(270);
      // add_target(13.6_in, 127_in, -225_deg);
      add_target(6_in, 150.85_in, -225_deg);
      wait(500);
      score_queue = 1;
      wait(200);
      targets.pop();
      wait(10);
      // wait(200);
      // add_target(32_in, 109_in, -180_deg);
      add_target(32_in, 118_in, -225_deg);
      wait(10);
      WAIT_UNTIL(final_target_reached)
      score_queue = 0;
    },
    [](){
      target_position_enabled = false;
    },
    {&auton_group});

Macro home_row_two(
    [&](){
      chassis->setState({15.7416_in, 31.4911_in, -90_deg});

      move_settings.start_output = 100;
      move_settings.end_output = 20;
      
      add_target(26.319_in, 26.319_in, -90_deg);
      add_target(26.319_in, 26.319_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 10;
      wait(400);
      top_roller_smart.add_target(45, 30);
      add_target(20.4_in, 20.4_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      wait(400);
      intake_queue = 0;
      intakes_back.start();
      wait(300);
      add_target(16_in, 16_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      add_target(13.6_in, 13.6_in, -135_deg);
      wait(200);
      score_queue = 1;
      wait(200);
      targets.pop();
      // add_target(30_in, 30_in, -135_deg);
      add_target(26_in, 72_in, -180_deg);
      // add_target(22_in, 72_in, -180_deg);

      WAIT_UNTIL(final_target_reached)
      add_target(17_in, 72_in, -180_deg);
      move_settings.start_output = 20;
      move_settings.end_output = 20;
      wait(500);
      score_queue = 1;
      wait(200);
      targets.pop();
      wait(10);
      move_settings.start_output = 100;
      move_settings.end_output = 50;
      add_target(32_in, 72_in, -180_deg);
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
      chassis->setState({15.7416_in, 31.4911_in, -90_deg});

      add_target(26.319_in, 26.319_in, -90_deg);
      add_target(26.319_in, 26.319_in, -135_deg);
      add_target(16_in, 16_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      add_target(13.6_in, 13.6_in, -135_deg);
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
      add_target(30_in, 30_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
    },
    [](){
      target_position_enabled = false;
    },
    {&auton_group});

Macro right_shawnton(
    [&](){
      chassis->setState({31.4911_in, 15.7416_in, -180_deg});

      add_target(26.319_in, 26.319_in, -180_deg);
      WAIT_UNTIL(final_target_reached)
      add_target(26.319_in, 26.319_in, -135_deg);
      wait(500);
      targets.pop();
      add_target(16_in, 16_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      add_target(13.6_in, 13.6_in, -135_deg);
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
      add_target(30_in, 30_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
    },
    [](){
      target_position_enabled = false;
    },
    {&auton_group});

void stop_scoring() {
  score_queue = 0;
  top_roller_smart.target_queue = {};
  // top_roller_smart.set_manual_speed(0, 0);
  // top_roller_smart.set_manual_speed(1, 0);
  top_roller_smart.auto_speed = 0;
}

Macro skills_one(
    [&](){
      chassis->setState({13.491_in, 34.9911_in, 0_deg});

      move_settings.start_output = 100;
      move_settings.end_output = 20;

      intake_queue = 1;
      add_target(18_in, 34.9911_in, 0_deg);
      add_target(5.8129_in, 5.8129_in, -135_deg, 17_in);
      wait(500);
      intakes_back.start();
      WAIT_UNTIL(final_target_reached)
      add_target(5.8129_in, 5.8129_in, -135_deg, 6_in);
      wait(500);
      score_queue = 1;
      wait(300);
      targets.pop();

      stop_scoring();
      add_target(5.9272_in, 70.3361_in, -180_deg, 20_in);
      WAIT_UNTIL(final_target_reached)
      add_target(5.9272_in, 70.3361_in, -180_deg, 6_in);
      wait(600);
      score_queue = 1;
      wait(400);
      targets.pop();

      stop_scoring();
      // move_settings.end_output = 100;
      add_target(23_in, 70.3361_in, -180_deg);
      add_target(23_in, 90_in, -270_deg);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      add_target(23_in, 117.18_in, -270_deg);
      // move_settings.end_output = 20;
      WAIT_UNTIL(final_target_reached)
      intakes_back.start();
      add_target(5.8129_in, 134.8593_in, -225_deg, 24.5_in);
      WAIT_UNTIL(final_target_reached)
      add_target(5.8129_in, 134.8593_in, -225_deg, 6_in);
      wait(1000);
      score_queue = 1;
      wait(400);
      targets.pop();

      stop_scoring();
      add_target(23_in, 117.18_in, -270_deg);
      WAIT_UNTIL(final_target_reached)
      intake_right.move_relative(180, 200);
      add_target(34.8361_in, 123.6722_in, -270_deg);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(1000);
      intakes_back.start();
      add_target(70.3361_in, 117.4624_in, -360_deg, 12_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      add_target(70.3361_in, 117.4624_in, -360_deg);
      wait(1000);
      intakes_back.start();
      add_target(70.3361_in, 117.4624_in, -270_deg);
      WAIT_UNTIL(final_target_reached)
      add_target(70.3361_in, 134.745_in, -270_deg, 6_in);
      wait(500);
      score_queue = 2;
      wait(700);
      targets.pop();

      stop_scoring();
      add_target(70.3361_in, 117.4624_in, -270_deg);
      WAIT_UNTIL(final_target_reached)
      intake_right.move_relative(180, 200);
      add_target(105.8361_in, 123.6722_in, -270_deg);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(1000);
      intakes_back.start();
      add_target(134.8593_in, 134.8593_in, -315_deg, 17_in);
      WAIT_UNTIL(final_target_reached)
      add_target(134.8593_in, 134.8593_in, -315_deg, 6_in);
      wait(300);
      score_queue = 1;
      wait(300);
      targets.pop();

      stop_scoring();
      add_target(118_in, 125_in, -360_deg);
      add_target(118_in, 125_in, -450_deg);
      add_target(117.6361_in, 105.6361_in, -450_deg, 12_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      add_target(117.6361_in, 70.3361_in, -450_deg);
      wait(1000);
      intakes_back.start();
      add_target(117.6361_in, 70.3361_in, -360_deg);
      WAIT_UNTIL(final_target_reached)
      add_target(134.745_in,  70.3361_in, -360_deg, 6_in);
      wait(1000);
      score_queue = 1;
      wait(200);
      targets.pop();

      stop_scoring();
      add_target(117.6361_in, 70.3361_in, -360_deg);
      add_target(117.6361_in, 35.0361_in, -450_deg, 12_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      add_target(117.1816_in, 23.4906_in, -450_deg);
      WAIT_UNTIL(final_target_reached)
      intakes_back.start();
      add_target(134.8593_in, 5.8129_in, -405_deg, 24.5_in);
      WAIT_UNTIL(final_target_reached)
      add_target(134.8593_in, 5.8129_in, -405_deg, 6_in);
      wait(700);
      score_queue = 1;
      wait(400);
      targets.pop();

      stop_scoring();
      add_target(117.1816_in, 23.4906_in, -450_deg);
      WAIT_UNTIL(final_target_reached)
      intake_right.move_relative(180, 200);
      add_target(105.8361_in, 3.3361_in, -450_deg , 13_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(1000);
      intakes_back.start();
      add_target(70.3361_in, 23.3361_in, -540_deg, 11_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      add_target(70.3361_in, 23.3361_in, -540_deg);
      wait(1000);
      intakes_back.start();
      add_target(70.3361_in, 23.3361_in, -450_deg);
      WAIT_UNTIL(final_target_reached)
      add_target(70.3361_in, 5.9272_in, -450_deg, 6_in);
      wait(600);
      score_queue = 2;
      wait(1700);
      targets.pop();

      stop_scoring();
      add_target(70.3361_in, 23.3361_in, -450_deg);
      add_target(70.3361_in, 23.3361_in, -270_deg);
      add_target(70.3361_in, 46.8361_in, -270_deg, 10_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(1000);
      add_target(70.3361_in, 70.3361_in, -270_deg, 16_in);
      WAIT_UNTIL(final_target_reached)
      add_target(70.3361_in, 70.3361_in, -270_deg, 6_in);
      wait(300);
      intake_queue = 50;
      wait(200);
      targets.pop();

      button_strafe = 10;
      button_turn = -6.84;
      button_forward = 3;
      wait(2000);
      button_strafe = -10;
      button_turn = 6.84;
      button_forward = 3;
      wait(3000);
      intake_queue = 1;
      score_queue = 1;
      button_strafe = 10;
      button_turn = -6.84;
      button_forward = 3;
      // wait(4000);
      WAIT_UNTIL(intake_queue == 0)
      button_strafe = 0;
      button_turn = 0;
      button_forward = 0;

      intakes_back.start();
    },
    [](){
      button_strafe = 0;
      button_turn = 0;
      button_forward = 0;
      target_position_enabled = false;
    },
    {&auton_group});

Macro skills_two(
    [&](){
      chassis->setState({13.491_in, 34.9911_in, 0_deg});

      // move_settings.start_output = 100;
      // move_settings.end_output = 20;

      // intake_queue = 1;
      add_target(18_in, 34.9911_in, 0_deg);
      add_target(goal_1, -135_deg, 17_in);
      wait_until_final_target_reached();
      drive_to_goal(goal_1);

      // add_target(5.8129_in, 5.8129_in, -135_deg, 6_in);
      // wait(500);
      // score_queue = 1;
      // wait(300);
      // targets.pop();

      // stop_scoring();
      // add_target(5.9272_in, 70.3361_in, -180_deg, 20_in);
      // wait_until_final_target_reached();
      // add_target(5.9272_in, 70.3361_in, -180_deg, 6_in);
      // wait(600);
      // score_queue = 1;
      // wait(400);
      // targets.pop();

      // stop_scoring();
      // // move_settings.end_output = 100;
      // add_target(23_in, 70.3361_in, -180_deg);
      // add_target(23_in, 90_in, -270_deg);
      // wait_until_final_target_reached();
      // intake_queue = 1;
      // add_target(23_in, 117.18_in, -270_deg);
      // // move_settings.end_output = 20;
      // wait_until_final_target_reached();
      // intakes_back.start();
      // add_target(5.8129_in, 134.8593_in, -225_deg, 24.5_in);
      // wait_until_final_target_reached();
      // add_target(5.8129_in, 134.8593_in, -225_deg, 6_in);
      // wait(1000);
      // score_queue = 1;
      // wait(400);
      // targets.pop();

      // stop_scoring();
      // add_target(23_in, 117.18_in, -270_deg);
      // wait_until_final_target_reached();
      // intake_right.move_relative(180, 200);
      // add_target(34.8361_in, 123.6722_in, -270_deg);
      // wait_until_final_target_reached();
      // intake_queue = 1;
      // wait(1000);
      // intakes_back.start();
      // add_target(70.3361_in, 117.4624_in, -360_deg, 12_in);
      // wait_until_final_target_reached();
      // intake_queue = 1;
      // add_target(70.3361_in, 117.4624_in, -360_deg);
      // wait(1000);
      // intakes_back.start();
      // add_target(70.3361_in, 117.4624_in, -270_deg);
      // wait_until_final_target_reached();
      // add_target(70.3361_in, 134.745_in, -270_deg, 6_in);
      // wait(500);
      // score_queue = 2;
      // wait(700);
      // targets.pop();

      // stop_scoring();
      // add_target(70.3361_in, 117.4624_in, -270_deg);
      // wait_until_final_target_reached();
      // intake_right.move_relative(180, 200);
      // add_target(105.8361_in, 123.6722_in, -270_deg);
      // wait_until_final_target_reached();
      // intake_queue = 1;
      // wait(1000);
      // intakes_back.start();
      // add_target(134.8593_in, 134.8593_in, -315_deg, 17_in);
      // wait_until_final_target_reached();
      // add_target(134.8593_in, 134.8593_in, -315_deg, 6_in);
      // wait(300);
      // score_queue = 1;
      // wait(300);
      // targets.pop();

      // stop_scoring();
      // add_target(118_in, 125_in, -360_deg);
      // add_target(118_in, 125_in, -450_deg);
      // add_target(117.6361_in, 105.6361_in, -450_deg, 12_in);
      // wait_until_final_target_reached();
      // intake_queue = 1;
      // add_target(117.6361_in, 70.3361_in, -450_deg);
      // wait(1000);
      // intakes_back.start();
      // add_target(117.6361_in, 70.3361_in, -360_deg);
      // wait_until_final_target_reached();
      // add_target(134.745_in,  70.3361_in, -360_deg, 6_in);
      // wait(1000);
      // score_queue = 1;
      // wait(200);
      // targets.pop();

      // stop_scoring();
      // add_target(117.6361_in, 70.3361_in, -360_deg);
      // add_target(117.6361_in, 35.0361_in, -450_deg, 12_in);
      // wait_until_final_target_reached();
      // intake_queue = 1;
      // add_target(117.1816_in, 23.4906_in, -450_deg);
      // wait_until_final_target_reached();
      // intakes_back.start();
      // add_target(134.8593_in, 5.8129_in, -405_deg, 24.5_in);
      // wait_until_final_target_reached();
      // add_target(134.8593_in, 5.8129_in, -405_deg, 6_in);
      // wait(700);
      // score_queue = 1;
      // wait(400);
      // targets.pop();

      // stop_scoring();
      // add_target(117.1816_in, 23.4906_in, -450_deg);
      // wait_until_final_target_reached();
      // intake_right.move_relative(180, 200);
      // add_target(105.8361_in, 3.3361_in, -450_deg , 13_in);
      // wait_until_final_target_reached();
      // intake_queue = 1;
      // wait(1000);
      // intakes_back.start();
      // add_target(70.3361_in, 23.3361_in, -540_deg, 11_in);
      // wait_until_final_target_reached();
      // intake_queue = 1;
      // add_target(70.3361_in, 23.3361_in, -540_deg);
      // wait(1000);
      // intakes_back.start();
      // add_target(70.3361_in, 23.3361_in, -450_deg);
      // wait_until_final_target_reached();
      // add_target(70.3361_in, 5.9272_in, -450_deg, 6_in);
      // wait(600);
      // score_queue = 2;
      // wait(1700);
      // targets.pop();

      // stop_scoring();
      // add_target(70.3361_in, 23.3361_in, -450_deg);
      // add_target(70.3361_in, 23.3361_in, -270_deg);
      // add_target(70.3361_in, 46.8361_in, -270_deg, 10_in);
      // wait_until_final_target_reached();
      // intake_queue = 1;
      // wait(1000);
      // add_target(70.3361_in, 70.3361_in, -270_deg, 16_in);
      // wait_until_final_target_reached();
      // add_target(70.3361_in, 70.3361_in, -270_deg, 6_in);
      // wait(300);
      // intake_queue = 50;
      // wait(200);
      // targets.pop();

      // button_strafe = 10;
      // button_turn = -6.84;
      // button_forward = 3;
      // wait(2000);
      // button_strafe = -10;
      // button_turn = 6.84;
      // button_forward = 3;
      // wait(3000);
      // intake_queue = 1;
      // score_queue = 1;
      // button_strafe = 10;
      // button_turn = -6.84;
      // button_forward = 3;
      // // wait(4000);
      // WAIT_UNTIL(intake_queue == 0)
      // button_strafe = 0;
      // button_turn = 0;
      // button_forward = 0;

      // intakes_back.start();
    },
    [](){
      button_strafe = 0;
      button_turn = 0;
      button_forward = 0;
      target_position_enabled = false;
    },
    {&auton_group});

Macro shawnton_three(
    [&](){
      chassis->setState({15.7416_in, 109.181_in, 90_deg});

      add_target(23.49_in, 117.18_in, 90_deg);
      add_target(5.8129_in, 134.8593_in, 135_deg, 25_in);
      // add_target(5.8129_in, 134.8593_in, 135_deg, 16_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      add_target(5.8129_in, 134.8593_in, 135_deg, 16_in);
      // wait(800);
      // intakes_back.start();
      // wait(400);
      WAIT_UNTIL(final_target_reached)
      add_target(5.8129_in, 134.8593_in, 135_deg, 6_in);
      intake_queue++;
      wait(500);
      // score_queue = 1;
      top_roller_smart.set_manual_speed(1, 600);
      wait(200);
      top_roller_smart.set_manual_speed(1, 0);
      targets.pop();
      stop_scoring();


      add_target(5.8129_in, 134.8593_in, 135_deg, 20_in);
      add_target(56.193_in, 124.602_in, 0_deg);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      // wait(500);
      add_target(70.3361_in, 134.745_in, 40_deg, 20_in);
      WAIT_UNTIL(final_target_reached)
      add_target(70.3361_in, 134.745_in, 40_deg, 6_in);
      wait(800);
      targets.pop();

      add_target(70.3361_in, 134.745_in, 30_deg, 20_in);
      WAIT_UNTIL(final_target_reached)
      intakes_back.start();
      wait(200);
      add_target(70.3361_in, 134.745_in, 30_deg, 6_in);
      wait(800);
      score_queue = 1;
      wait(200);
      targets.pop();
      stop_scoring();


      // add_target(41.393_in, 114.745_in, 40_deg);
      add_target(41.393_in, 114.745_in, 0_deg);
      add_target(41.393_in, 70.3361_in, 0_deg);
      // add_target(70.3361_in, 70.3361_in, 0_deg, 16_in);
      // wait(300);
      // intake_queue = 1;
      // wait(500);
      // intakes_back.start();
      // wait(100);
      WAIT_UNTIL(final_target_reached)
      add_target(70.3361_in, 70.3361_in, 0_deg, 6_in);
      wait(1200);
      score_queue = 1;
      wait(500);
      intake_left.move_relative(50, 200);
      intake_right.move_relative(50, 200);
      wait(800);
      intake_left.move_relative(-50, 200);
      intake_right.move_relative(-50, 200);
      wait(300);
      targets.pop();
      WAIT_UNTIL(final_target_reached)
    },
    [](){
      target_position_enabled = false;
    },

    {&auton_group});

Macro shawnton_cycle(
    [&](){
      chassis->setState({15.7416_in, 109.181_in, 90_deg});

      add_target(23.49_in, 117.18_in, 90_deg);
      add_target(5.8129_in, 134.8593_in, 135_deg, 25_in);
      add_target(5.8129_in, 134.8593_in, 135_deg, 16_in);
      // add_target(5.8129_in, 134.8593_in, 135_deg, 16_in);
      wait(10);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(800);
      intake_left = 0;
      intake_right = 0;
      intake_left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      intake_right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      add_target(5.8129_in, 134.8593_in, 135_deg, 25_in);
      add_target(5.8129_in, 134.8593_in, 135_deg, 16_in);
      wait(10);
      WAIT_UNTIL(final_target_reached)
      add_target(5.8129_in, 134.8593_in, 135_deg, 6_in);
      wait(300);
      score_queue = 2;
      wait(1000);
      targets.pop();

      intake_queue = 1;
      wait(800);
      intake_left = 0;
      intake_right = 0;
      intake_left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      intake_right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      intake_queue = 1;
      wait(800);
      intake_left = 0;
      intake_right = 0;
      intake_left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      intake_right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      score_queue = 1;
      intakes_back.start();
      wait(200);

      add_target(5.8129_in, 134.8593_in, 135_deg, 20_in);
      wait(10);
      
      WAIT_UNTIL(final_target_reached)
    },
    [](){
      target_position_enabled = false;
    },
    {&auton_group});

} // namespace autonroutines