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
  QLength x_offset = cos(tracking_coords.theta.convert(radian)) * TRACKING_ORIGIN_OFFSET;
  QLength y_offset = sin(tracking_coords.theta.convert(radian)) * TRACKING_ORIGIN_OFFSET;

  return {tracking_coords.x + x_offset, tracking_coords.y + y_offset, tracking_coords.theta};
}

OdomState robot_to_tracking_coords (OdomState robot_coords) {
  QLength x_offset = cos(robot_coords.theta.convert(radian)) * -TRACKING_ORIGIN_OFFSET;
  QLength y_offset = sin(robot_coords.theta.convert(radian)) * -TRACKING_ORIGIN_OFFSET;

  return {robot_coords.x + x_offset, robot_coords.y + y_offset, robot_coords.theta};
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
RampMathSettings move_settings = {20, 100, 20, 0.1, 0.1};
RampMathSettings turn_settings = {10, 50, 10, 0.1, 0.1};

double forward = 0;
double strafe  = 0;
double turn    = 0;

void addPositionTarget(QLength x, QLength y, QAngle theta, QLength offset) {
  QLength x_offset = cos(theta.convert(radian)) * offset;
  QLength y_offset = sin(theta.convert(radian)) * offset;
  targets.push({x + x_offset, y + y_offset, theta});
  target_position_enabled = true;
  final_target_reached = false;
  controllermenu::master_print_array[2] = "targets.size(): " + std::to_string(targets.size());
};


void update() {
  controllermenu::master_print_array[0] = "TDR: " + std::to_string(target_distance_reached) + " THR: " + std::to_string(target_heading_reached);
  controllermenu::master_print_array[1] = "targets.size(): " + std::to_string(targets.size());
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

    double forward = drivetoposition::forward + master.get_analog(ANALOG_RIGHT_Y) * 0.787401574803;
    double strafe  = drivetoposition::strafe  + master.get_analog(ANALOG_RIGHT_X) * 0.787401574803;
    double temp_turn    = master.get_analog(ANALOG_LEFT_X) * 0.787401574803;
    double turn    = drivetoposition::turn    + pow(abs(temp_turn / 100), 1.8) * 100 * sgn(temp_turn);
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
























controllerbuttons::Macro drive_test(
    [&](){
      chassis->setState(robot_to_tracking_coords({0_in, 0_in, 0_deg}));
      // drivetoposition::addPositionTarget(15_in, 15_in, -90_deg);
      // pros::delay(3000);
      // drivetoposition::addPositionTarget(15_in, 15_in, 0_deg);
      // pros::delay(3000);
      // drivetoposition::addPositionTarget(15_in, 0_in, 0_deg);
      // pros::delay(3000);
      drivetoposition::addPositionTarget(0_in, 0_in, 0_deg);
      pros::delay(500);
      drivetoposition::addPositionTarget(0_in, 0_in, 360_deg);
      pros::delay(500);
      drivetoposition::addPositionTarget(0_in, 0_in, 0_deg);
      pros::delay(3000);
      // WAIT_UNTIL(drivetoposition::final_target_reached);
    },
    [](){
      drivetoposition::target_position_enabled = false;
    },
    {&auton_group});

controllerbuttons::Macro left_home_row(
    [&](){
      using namespace robotfunctions;
      using namespace drivetoposition;
      using namespace controllerbuttons;
      chassis->setState(robot_to_tracking_coords({15.7411_in, 26.3036_in, -90_deg}));
      using namespace rollers;
      // balls_in_robot = 1;
      addPositionTarget(26.3_in, 26.3_in, -90_deg);
      addPositionTarget(26.3_in, 26.3_in, -135_deg);
      addPositionTarget(24_in, 24_in, -135_deg);
      WAIT_UNTIL(drivetoposition::targets.size() == 1)
      // intake_queue++;
      // WAIT_UNTIL(intake_queue == 0);
      wait(500);
      // intakes_back.start();
      // WAIT_UNTIL(!intakes_back.is_running());
      wait(500);
      addPositionTarget(26.3_in, 26.3_in, -135_deg);
      addPositionTarget(0_in, 0_in, -135_deg);
      WAIT_UNTIL(drivetoposition::targets.size() == 1)
      wait(1000);
      drivetoposition::targets.pop();
      // score_queue++;
      wait(500);
      addPositionTarget(30_in, 30_in, -180_deg);
      addPositionTarget(30_in, 70.3_in, -180_deg);
      addPositionTarget(0_in, 70.3_in, -180_deg);
      WAIT_UNTIL(drivetoposition::targets.size() == 1);
      wait(500);
      drivetoposition::targets.pop();
      // score_queue++;
      wait(500);
      addPositionTarget(40_in, 74_in, -180_deg);
      addPositionTarget(40_in, 112_in, -180_deg);
      addPositionTarget(40_in, 112_in, -225_deg);
      addPositionTarget(22_in, 138_in, -225_deg);
      WAIT_UNTIL(drivetoposition::targets.size() == 1);
      // intake_queue++;
      // WAIT_UNTIL(intake_queue == 0);
      wait(500);
      addPositionTarget(30_in, 120_in, -225_deg);
      // intakes_back.start();

      // WAIT_UNTIL(!intakes_back.is_running());
      wait(1000);
      addPositionTarget(6_in, 150.85_in, -225_deg);
      WAIT_UNTIL(drivetoposition::targets.size() == 1);
      wait(500);
      drivetoposition::targets.pop();
      // score_queue++;
      wait(500);
      addPositionTarget(30_in, 138_in, -225_deg);
    },
    [](){
    },
    {&auton_group});

controllerbuttons::Macro shawnton_right(
    [&](){
      // while (true) {
      //   driveToPosition(0_in, 0_in, 0_deg);
      //   controllerbuttons::wait(5);
      // }
      chassis->setState({0_in, 0_in, 0_deg});
      using namespace robotfunctions;
      using namespace drivetoposition;
      using namespace controllerbuttons;
      using namespace rollers;
      balls_in_robot = 1;
      addPositionTarget(0_in, -10.55_in, 0_deg);
      addPositionTarget(0_in, -10.55_in, 45_deg);
      WAIT_UNTIL(drivetoposition::targets.size() == 1)
      intake_queue = 3;
      addPositionTarget(5_in, -5.55_in, 45_deg);
      // WAIT_UNTIL(intake_queue == 0);
      wait(1000);
      // intakes_back.start();
      // WAIT_UNTIL(!intakes_back.is_running());
      addPositionTarget(26.3_in, 15.74_in, 45_deg);
      WAIT_UNTIL(drivetoposition::targets.size() == 1)
      wait(1000);
      drivetoposition::targets.pop();
      score_queue = 3;
      wait(500);
      addPositionTarget(0_in, -10.55_in, 45_deg);
    },
    [](){
    },
    {&auton_group});

void set_callbacks() {
  using namespace controllerbuttons;
  button_handler.master.a.pressed.set_macro(drive_test);
  button_handler.master.y.pressed.set_macro(left_home_row);
  button_handler.master.b.pressed.set([&](){ drive_test.terminate(); });
}

} // namespace autondrive