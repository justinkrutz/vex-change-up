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

// struct Position {
//   QLength x = 0_in;
//   QLength y = 0_in;
//   QAngle theta = 0_deg;
//   bool is_new = true;
//   OdomState starting_state;
// };


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
// target pos
// at start:
//   starting pos

// namespace positiontarget{
//   QLength x;
//   QLength y;
//   QAngle theta;
//   QLength offset;
// } // positiontarget

Target::Target(QLength x, QLength y, QAngle theta) : x(x), y(y), theta(theta) {};

double Target::forward = 0;
double Target::strafe = 0;
double Target::turn = 0;

void Target::init_if_new() {
  if (is_new) {
    is_new = false;
    starting_state = robot_state();
  }
}

void Target::drive() {
    // Point target_point{x, y};
    // QAngle direction = OdomMath::computeAngleToPoint(target_point, robot_state());

    // OdomState target_state{x, y, theta};
    // Point starting_point{starting_state.x, starting_state.y};
    // QLength traveled_distance = OdomMath::computeDistanceToPoint(starting_point, robot_state());
    // QLength total_distance = OdomMath::computeDistanceToPoint(starting_point, target_state);
    // // auto [magnitude_real, direction_real] = OdomMath::computeDistanceAndAngleToPoint(starting_point, robot_state());
    // // auto [magnitude_target, direction_target] = OdomMath::computeDistanceAndAngleToPoint(starting_point, target_state);
    // // auto [start_magnitude, start_direction] = OdomMath::computeDistanceAndAngleToPoint(target_point, starting_position);

    // // move_settings.start_output = std::max(20.0, sqrt(forward * forward + strafe * strafe));
    // double move_speed = rampMath(traveled_distance.convert(inch), total_distance.convert(inch), move_settings);
    // QAngle angle_turned = robot_state().theta - starting_state.theta;
    // QAngle total_angle = theta - starting_state.theta;
    // // double turn_speed = sgn(total_angle.convert(radian)) * rampMath(angle_turned.convert(radian), total_angle.convert(radian), turn_settings);
    // // double move_speed = std::min(100.0, magnitude.convert(inch)*10);
    // // double move_speed = 0;
    // double turn_speed = std::min(100.0, 100 * (theta - robot_state().theta).convert(radian));
    // forward = move_speed * cos(direction.convert(radian));
    // strafe  = move_speed * sin(direction.convert(radian));
    // turn    = turn_speed;
    // // controllermenu::master_print_array[0] = "dir: " + std::to_string(direction.convert(degree));
    // // controllermenu::master_print_array[1] = "mag: " + std::to_string(magnitude.convert(inch));
}

namespace drivetoposition {
  std::queue<Target> targets;
  bool targetPositionEnabled = false;
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
    targetPositionEnabled = true;
    final_target_reached = false;
  };

  void driveToPosition() {
    Target &target = targets.front();
    Point target_point{target.x, target.y};
    QAngle direction = OdomMath::computeAngleToPoint(target_point, robot_state());

    OdomState target_state{target.x, target.y, target.theta};
    Point starting_point{target.starting_state.x, target.starting_state.y};
    QLength traveled_distance = OdomMath::computeDistanceToPoint(starting_point, robot_state());
    QLength total_distance = OdomMath::computeDistanceToPoint(starting_point, target_state);
    // auto [magnitude_real, direction_real] = OdomMath::computeDistanceAndAngleToPoint(starting_point, robot_state());
    // auto [magnitude_target, direction_target] = OdomMath::computeDistanceAndAngleToPoint(starting_point, target_state);
    // auto [start_magnitude, start_direction] = OdomMath::computeDistanceAndAngleToPoint(target_point, starting_position);

    // move_settings.start_output = std::max(20.0, sqrt(forward * forward + strafe * strafe));
    // double move_speed = rampMath(traveled_distance.convert(inch), total_distance.convert(inch), move_settings);
    double move_speed = 20;
    QAngle angle_turned = robot_state().theta - target.starting_state.theta;
    QAngle total_angle = target.theta - target.starting_state.theta;
    // double turn_speed = sgn(total_angle.convert(radian)) * rampMath(angle_turned.convert(radian), total_angle.convert(radian), turn_settings);
    // double move_speed = std::min(100.0, magnitude.convert(inch)*10);
    // double move_speed = 0;
    double turn_speed = std::min(50.0, 50 * (target.theta - robot_state().theta).convert(radian));
    forward = move_speed * cos(direction.convert(radian));
    strafe  = move_speed * sin(direction.convert(radian));
    turn    = turn_speed;
    // controllermenu::master_print_array[0] = "dir: " + std::to_string(direction.convert(degree));
    // controllermenu::master_print_array[1] = "mag: " + std::to_string(magnitude.convert(inch));
  }

    void holdPosition() {
    Target target = targets.front();
    Point target_point{target.x, target.y};
    auto [magnitude, direction] = OdomMath::computeDistanceAndAngleToPoint(target_point, robot_state());

    OdomState target_state{target.x, target.y, target.theta};
    Point starting_point{target.starting_state.x, target.starting_state.y};
    auto [magnitude_target, direction_target] = OdomMath::computeDistanceAndAngleToPoint(starting_point, target_state);
    auto [magnitude_real, direction_real] = OdomMath::computeDistanceAndAngleToPoint(starting_point, robot_state());
    // auto [start_magnitude, start_direction] = OdomMath::computeDistanceAndAngleToPoint(target_point, starting_position);

    move_settings.start_output = std::max(20.0, sqrt(forward * forward + strafe * strafe));
    // double move_speed = rampMath(magnitude_real.convert(inch), magnitude_target.convert(inch), move_settings);
    // double turn_speed = rampMath(direction.convert(radian), start_direction.convert(radian), turn_settings);
    double move_speed = std::min(100.0, magnitude.convert(inch)*5);
    // double move_speed = 0;
    double turn_speed = std::min(100.0, 100 * (target.theta - robot_state().theta).convert(radian));
    forward = move_speed * cos(direction.convert(radian));
    strafe  = move_speed * sin(direction.convert(radian));
    turn    = turn_speed;
    // controllermenu::master_print_array[0] = "dir: " + std::to_string(direction.convert(degree));
    // controllermenu::master_print_array[1] = "mag: " + std::to_string(magnitude.convert(inch));
  }

  void update() {
    // controllermenu::master_print_array[2] = "targets: " + std::to_string(targets.size());
    if (targetPositionEnabled && targets.size() > 0) {
      Target &target = targets.front();
      target.init_if_new();
      if (targets.size() > 1) {
        // move_settings.end_output = 100;
        OdomState target_state{target.x, target.y, target.theta};
        Point starting_point{target.starting_state.x, target.starting_state.y};
        auto [magnitude_target, direction_target] = OdomMath::computeDistanceAndAngleToPoint(starting_point, target_state);
        auto [magnitude_real, direction_real] = OdomMath::computeDistanceAndAngleToPoint(starting_point, robot_state());
        driveToPosition();
        if (magnitude_real >= magnitude_target) {
          target_distance_reached = true;
        }
        if (fabs(robot_state().theta.convert(radian) - target_state.theta.convert(radian)) < 5*degreeToRadian) {
        target_heading_reached = true;
        }
        if (target_heading_reached && target_distance_reached) {
          targets.pop();
          target_heading_reached = false;
          target_distance_reached = false;
        }
      } else {
        holdPosition();
        final_target_reached = true;
        // move_settings.end_output = 1;
      }
    } else {
      forward = 0;
      strafe = 0;
      turn = 0;
      targets = {};
    }
  }
};

void motor_task()
{
  std::shared_ptr<AbstractMotor> drive_fl = x_model->getTopLeftMotor();
  std::shared_ptr<AbstractMotor> drive_fr = x_model->getTopRightMotor();
  std::shared_ptr<AbstractMotor> drive_bl = x_model->getBottomLeftMotor();
  std::shared_ptr<AbstractMotor> drive_br = x_model->getBottomRightMotor();

  Slew drive_fl_slew(DRIVER_SLEW);
  Slew drive_fr_slew(DRIVER_SLEW);
  Slew drive_bl_slew(DRIVER_SLEW);
  Slew drive_br_slew(DRIVER_SLEW);

  while(1)
  {

    // double ctr_f = master.get_analog(ANALOG_RIGHT_Y) * 0.787401574803;
    // double ctr_s = -master.get_analog(ANALOG_RIGHT_X) * 0.787401574803;
    // double ctr_t = master.get_analog(ANALOG_LEFT_X) * 0.787401574803;
    // double theta = 0;
    // if (ctr_f != 0) {
    //   theta = atan(ctr_s / ctr_f);
    // } else {
    //   theta = 90 * degreeToRadian * sgn(ctr_s);
    // }
    // double move_m = sqrt(pow(ctr_f, 2) + pow(ctr_s, 2)) * sgn(ctr_f);
    // double forward = move_m * cos(robot_state().theta.convert(radian) + theta);
    // double strafe  = move_m * -sin(robot_state().theta.convert(radian) + theta);
    // double turn    = ctr_t;
    drivetoposition::update();

    double forward = drivetoposition::forward + master.get_analog(ANALOG_RIGHT_Y) * 0.787401574803;
    double strafe  = drivetoposition::strafe  + master.get_analog(ANALOG_RIGHT_X) * 0.787401574803;
    // double turn    = drivetoposition::turn    + master.get_analog(ANALOG_LEFT_X) * 0.787401574803 * ((master.get_analog(ANALOG_LEFT_Y) * 0.787401574803) / 100 + 1.1);
    double temp_turn    = master.get_analog(ANALOG_LEFT_X) * 0.787401574803;
    double turn    = drivetoposition::turn    + pow(abs(temp_turn / 100), 1.8) * 100 * sgn(temp_turn);
    double m = std::min(1.0, 100 / (fabs(forward) + fabs(strafe) + fabs(turn)));


    double drive_fl_value = drive_fl_slew.new_value((forward + strafe + turn) * 2 * m);
    double drive_fr_value = drive_fr_slew.new_value((forward - strafe - turn) * 2 * m);
    double drive_bl_value = drive_bl_slew.new_value((forward - strafe + turn) * 2 * m);
    double drive_br_value = drive_br_slew.new_value((forward + strafe - turn) * 2 * m);

    drive_fl->moveVelocity(drive_fl_value);
    drive_fr->moveVelocity(drive_fr_value);
    drive_bl->moveVelocity(drive_bl_value);
    drive_br->moveVelocity(drive_br_value);

    pros::delay(5);
  }
}

controllerbuttons::Macro count_up(
    [](){
      printf("start\n");
      for (int i = 0; i < 50; i++) {
        printf("Up %d\n", i);
        controllerbuttons::wait(20);
      }
    },
    [](){

    },
    {&auton_group});


// Test function that prints to the terminal.
void single_use_button() {
  printf("single_use_button\n");
}




// #define WAIT_UNTIL(condition) \
// while (!(condition)) {        \
// pros::delay(5);               \
// }

controllerbuttons::Macro drive_test(
    [&](){
      chassis->setState(robot_to_tracking_coords({0_in, 0_in, 0_deg}));
      drivetoposition::addPositionTarget(15_in, 15_in, -90_deg);
      drivetoposition::addPositionTarget(15_in, 15_in, 0_deg);
      drivetoposition::addPositionTarget(15_in, 0_in, 0_deg);
      drivetoposition::addPositionTarget(0_in, 0_in, 0_deg);
      drivetoposition::addPositionTarget(0_in, 0_in, 360_deg);
      WAIT_UNTIL(drivetoposition::final_target_reached);
    },
    [](){
      drivetoposition::targetPositionEnabled = false;
    },
    {&auton_group});

controllerbuttons::Macro main_auton(
    [&](){
      // while (true) {
      //   driveToPosition(0_in, 0_in, 0_deg);
      //   controllerbuttons::wait(5);
      // }
      using namespace robotfunctions;
      using namespace drivetoposition;
      using namespace controllerbuttons;
      using namespace rollers;
      balls_in_robot = 1;
      addPositionTarget(26.3_in, 26.3_in, -90_deg);
      addPositionTarget(26.3_in, 26.3_in, -135_deg);
      addPositionTarget(24_in, 24_in, -135_deg);
      WAIT_UNTIL(drivetoposition::targets.size() == 1)
      intake_queue++;
      // WAIT_UNTIL(intake_queue == 0);
      wait(500);
      intakes_back.start();
      // WAIT_UNTIL(!intakes_back.is_running());
      wait(500);
      addPositionTarget(26.3_in, 26.3_in, -135_deg);
      addPositionTarget(0_in, 0_in, -135_deg);
      WAIT_UNTIL(drivetoposition::targets.size() == 1)
      wait(1000);
      drivetoposition::targets.pop();
      score_queue++;
      wait(500);
      addPositionTarget(30_in, 30_in, -180_deg);
      addPositionTarget(30_in, 70.3_in, -180_deg);
      addPositionTarget(0_in, 70.3_in, -180_deg);
      WAIT_UNTIL(drivetoposition::targets.size() == 1);
      wait(500);
      drivetoposition::targets.pop();
      score_queue++;
      wait(500);
      addPositionTarget(40_in, 74_in, -180_deg);
      addPositionTarget(40_in, 112_in, -180_deg);
      addPositionTarget(40_in, 112_in, -225_deg);
      addPositionTarget(22_in, 138_in, -225_deg);
      WAIT_UNTIL(drivetoposition::targets.size() == 1);
      intake_queue++;
      // WAIT_UNTIL(intake_queue == 0);
      wait(500);
      addPositionTarget(30_in, 120_in, -225_deg);
      intakes_back.start();

      // WAIT_UNTIL(!intakes_back.is_running());
      wait(1000);
      addPositionTarget(6_in, 150.85_in, -225_deg);
      WAIT_UNTIL(drivetoposition::targets.size() == 1);
      wait(500);
      drivetoposition::targets.pop();
      score_queue++;
      wait(500);
      addPositionTarget(30_in, 138_in, -225_deg);
    },
    [](){
    },
    {&auton_group});

controllerbuttons::Macro shawnton(
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
  button_handler.master.b.pressed.set([&](){ drive_test.terminate(); });
}

} // namespace autondrive