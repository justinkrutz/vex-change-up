#include "main.h"

#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"
#include "auton-from-sd.h"
#include <stdio.h>
#include <complex.h>

controllerbuttons::MacroGroup test_group;
controllerbuttons::MacroGroup intake_group;

template <typename T> int sgn(T &&val) {
  if (val < 0) {
    return -1;
  } else {
    return 1;
  }
}

struct rampMathSettings {
  int start_output;
  int mid_output;
  int end_output;
  double ramp_up_p;
  double ramp_down_p;
};

int rampMath(double input, double total_range, rampMathSettings s) {
  input = fabs(input);
  total_range = fabs(total_range);
  int start_output = abs(s.start_output);
  int mid_output = abs(s.mid_output);
  int end_output = abs(s.end_output);
  double ramp_up_p = fabs(s.ramp_up_p);
  double ramp_down_p = fabs(s.ramp_down_p);

  double ramp_up_range = (mid_output - start_output)*ramp_up_p;
  double ramp_down_range = (mid_output - end_output)*ramp_down_p;
  double ramp_range_multiplier = std::min(1.0, total_range / (ramp_up_range + ramp_down_range));
  if (start_output != mid_output && input < ramp_up_range * ramp_range_multiplier) {
    return input / ramp_up_p + start_output;
  } else if (end_output != mid_output && input > (total_range - ramp_down_range) * ramp_range_multiplier) {
    return (total_range - input) / ramp_down_p + end_output;
  }
  return mid_output;
}

bool targetPositionEnabled = false;

struct Position {
  QLength x = 0_in;
  QLength y = 0_in;
  QAngle theta = 0_deg;
  bool is_new = true;
  OdomState starting_state;
};

// target pos
// at start:
//   starting pos



struct Target {
  Position start;
  Position end;
};

std::queue<Target> targets = {};

// namespace positiontarget{
//   QLength x;
//   QLength y;
//   QAngle theta;
//   QLength offset;
// } // positiontarget

namespace drivetoposition {
  std::queue<Position> targets;
  bool targetPositionEnabled = false;

  OdomState starting_position;
  rampMathSettings move_settings = {20, 100, 20, 0.5, 0.5};
  rampMathSettings turn_settings = {20, 100, 20, 0.1, 0.1};

  double forward = 0;
  double strafe  = 0;
  double turn    = 0;

  void addPositionTarget(QLength x, QLength y, QAngle theta, QLength offset = 0_in) {
    QLength x_new = cos(theta.convert(radian)) * offset;
    QLength y_new = sin(theta.convert(radian)) * offset;
    targets.push({x + x_new, y + y_new, theta});
    targetPositionEnabled = true;
  };

  void driveToPosition() {
    Position target = targets.front();
    Point target_point{target.x, target.y};
    auto [magnitude, direction] = OdomMath::computeDistanceAndAngleToPoint(target_point, chassis->getState());

    OdomState target_state{target.x, target.y, target.theta};
    Point starting_point{target.starting_state.x, target.starting_state.y};
    auto [magnitude_real, direction_real] = OdomMath::computeDistanceAndAngleToPoint(starting_point, chassis->getState());
    auto [magnitude_target, direction_target] = OdomMath::computeDistanceAndAngleToPoint(starting_point, target_state);
    // auto [start_magnitude, start_direction] = OdomMath::computeDistanceAndAngleToPoint(target_point, starting_position);

    move_settings.start_output = std::max(20.0, sqrt(forward * forward + strafe * strafe));
    double move_speed = rampMath(magnitude_real.convert(inch), magnitude_target.convert(inch), move_settings);
    // double turn_speed = rampMath(direction.convert(radian), start_direction.convert(radian), turn_settings);
    // double move_speed = std::min(100.0, magnitude.convert(inch)*10);
    // double move_speed = 0;
    double turn_speed = std::min(100.0, 100 * (target.theta - chassis->getState().theta).convert(radian));
    forward = move_speed * cos(direction.convert(radian));
    strafe  = move_speed * sin(direction.convert(radian));
    turn    = turn_speed;
    // controllermenu::master_print_array[0] = "dir: " + std::to_string(direction.convert(degree));
    // controllermenu::master_print_array[1] = "mag: " + std::to_string(magnitude.convert(inch));
  }

    void holdPosition() {
    Position target = targets.front();
    Point target_point{target.x, target.y};
    auto [magnitude, direction] = OdomMath::computeDistanceAndAngleToPoint(target_point, chassis->getState());

    OdomState target_state{target.x, target.y, target.theta};
    Point starting_point{target.starting_state.x, target.starting_state.y};
    auto [magnitude_target, direction_target] = OdomMath::computeDistanceAndAngleToPoint(starting_point, target_state);
    auto [magnitude_real, direction_real] = OdomMath::computeDistanceAndAngleToPoint(starting_point, chassis->getState());
    // auto [start_magnitude, start_direction] = OdomMath::computeDistanceAndAngleToPoint(target_point, starting_position);

    move_settings.start_output = std::max(20.0, sqrt(forward * forward + strafe * strafe));
    // double move_speed = rampMath(magnitude_real.convert(inch), magnitude_target.convert(inch), move_settings);
    // double turn_speed = rampMath(direction.convert(radian), start_direction.convert(radian), turn_settings);
    double move_speed = std::min(100.0, magnitude.convert(inch)*5);
    // double move_speed = 0;
    double turn_speed = std::min(100.0, 100 * (target.theta - chassis->getState().theta).convert(radian));
    forward = move_speed * cos(direction.convert(radian));
    strafe  = move_speed * sin(direction.convert(radian));
    turn    = turn_speed;
    // controllermenu::master_print_array[0] = "dir: " + std::to_string(direction.convert(degree));
    // controllermenu::master_print_array[1] = "mag: " + std::to_string(magnitude.convert(inch));
  }

  void update() {
    // controllermenu::master_print_array[2] = "targets: " + std::to_string(targets.size());
    if (targetPositionEnabled) {
      if (targets.size() == 0) {
        forward = 0;
        strafe = 0;
        turn = 0;
      // } else if (targets.size() == 1) {
      //   // driveToPosition(targets.front(),
      //   //                 {100, 100, 20, 8, 8},
      //   //                 {100, 100, 20, 100, 100});
      //   driveToPosition();
      //   // move_settings = {100, 100, 20, 8, 8},
      //   // turn_settings = {100, 100, 20, 100, 100};
      } else {
        if (targets.front().is_new) {
          targets.front().is_new = false;
          targets.front().starting_state = chassis->getState();
        }
        Position target = targets.front();
        // move_settings = {100, 100, 100, 8, 8},
        // turn_settings = {100, 100, 100, 100, 100};

        if (targets.size() > 1) {
          // move_settings.end_output = 100;
          OdomState target_state{target.x, target.y, target.theta};
          Point starting_point{target.starting_state.x, target.starting_state.y};
          auto [magnitude_target, direction_target] = OdomMath::computeDistanceAndAngleToPoint(starting_point, target_state);
          auto [magnitude_real, direction_real] = OdomMath::computeDistanceAndAngleToPoint(starting_point, chassis->getState());
          driveToPosition();
          if (magnitude_real >= magnitude_target && fabs(chassis->getState().theta.getValue() - target_state.theta.getValue()) < 3*degreeToRadian) {
            targets.pop();
          }
        } else {
          holdPosition();
          // move_settings.end_output = 1;

        }
      }
    }
  }
};

#define DRIVER_SLEW 3
#define AUTON_SLEW 10

double slew(double new_value, double old_value, double slew_rate = AUTON_SLEW) {
  if (new_value > old_value + slew_rate)
    return old_value + slew_rate;
  if (new_value < old_value - slew_rate)
    return old_value - slew_rate;
  return new_value;
}

void motorTask()
{
  std::shared_ptr<AbstractMotor> drive_fl = x_model->getTopLeftMotor();
  std::shared_ptr<AbstractMotor> drive_fr = x_model->getTopRightMotor();
  std::shared_ptr<AbstractMotor> drive_bl = x_model->getBottomLeftMotor();
  std::shared_ptr<AbstractMotor> drive_br = x_model->getBottomRightMotor();

  double drive_fl_old = 0;
  double drive_fr_old = 0;
  double drive_bl_old = 0;
  double drive_br_old = 0;

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
    // double forward = move_m * cos(chassis->getState().theta.convert(radian) + theta);
    // double strafe  = move_m * -sin(chassis->getState().theta.convert(radian) + theta);
    // double turn    = ctr_t;
    if (pros::competition::is_autonomous()) {
      drivetoposition::update();
    } else {
      drivetoposition::forward = 0;
      drivetoposition::strafe = 0;
      drivetoposition::turn = 0;
    }


    double forward = drivetoposition::forward + master.get_analog(ANALOG_RIGHT_Y) * 0.787401574803;
    double strafe  = drivetoposition::strafe  + master.get_analog(ANALOG_RIGHT_X) * 0.787401574803;
    // double turn    = drivetoposition::turn    + master.get_analog(ANALOG_LEFT_X) * 0.787401574803 * ((master.get_analog(ANALOG_LEFT_Y) * 0.787401574803) / 100 + 1.1);
    double temp_turn    = master.get_analog(ANALOG_LEFT_X) * 0.787401574803;
    double turn    = drivetoposition::turn    + pow(abs(temp_turn / 100), 1.8) * 100 * sgn(temp_turn);
    double m = std::min(1.0, 100 / (fabs(forward) + fabs(strafe) + fabs(turn)));


    double drive_fl_value = slew((forward + strafe + turn) * 2 * m, drive_fl_old);
    double drive_fr_value = slew((forward - strafe - turn) * 2 * m, drive_fr_old);
    double drive_bl_value = slew((forward - strafe + turn) * 2 * m, drive_bl_old);
    double drive_br_value = slew((forward + strafe - turn) * 2 * m, drive_br_old);

    drive_fl->moveVelocity(drive_fl_value);
    drive_fr->moveVelocity(drive_fr_value);
    drive_bl->moveVelocity(drive_bl_value);
    drive_br->moveVelocity(drive_br_value);

    drive_fl_old = drive_fl_value;
    drive_fr_old = drive_fr_value;
    drive_bl_old = drive_bl_value;
    drive_br_old = drive_br_value;

    pros::delay(5);
  }
}


// void intakeBalls(int balls) {
// }

// void scoreBalls(int balls) {
// }


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
    {&test_group});


// Test function that prints to the terminal.
void single_use_button() {
  printf("single_use_button\n");
}




#define WAIT_UNTIL(condition) \
while (!(condition)) {        \
pros::delay(5);               \
}

controllerbuttons::Macro drive_test(
    [&](){
      drivetoposition::addPositionTarget(26.3_in, 26.3_in, -90_deg);
      drivetoposition::addPositionTarget(26.3_in, 26.3_in, -135_deg);
      drivetoposition::addPositionTarget(24_in, 24_in, -135_deg);
    },
    [](){
    },
    {&test_group});

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
    {&test_group});

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
    {&test_group});




void set_callbacks() {
  using namespace controllerbuttons;
  button_handler.master.a.pressed.set_macro(drive_test);
  button_handler.master.b.pressed.set([&](){ drive_test.terminate(); });
}
