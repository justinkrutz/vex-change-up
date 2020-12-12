#include "main.h"

#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"
#include "autonomous.h"
#include <stdio.h>
#include <complex.h>


namespace robotfunctions {
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

namespace DriveToPosition {
  std::queue<Position> targets;
  bool targetPositionEnabled = false;
  
  OdomState starting_position;
  rampMathSettings move_settings = {20, 100, 20, 0.1, 0.1};
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
    controllermenu::controller_print_array[0] = "dir: " + std::to_string(direction.convert(degree));
    controllermenu::controller_print_array[1] = "mag: " + std::to_string(magnitude.convert(inch));
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
    controllermenu::controller_print_array[0] = "dir: " + std::to_string(direction.convert(degree));
    controllermenu::controller_print_array[1] = "mag: " + std::to_string(magnitude.convert(inch));
  }

  void update() {
    // controllermenu::controller_print_array[2] = "targets: " + std::to_string(targets.size());
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
          if (magnitude_real >= magnitude_target) {
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





void motorTask()
{
  std::shared_ptr<AbstractMotor> drive_fl = x_model->getTopLeftMotor();
  std::shared_ptr<AbstractMotor> drive_fr = x_model->getTopRightMotor();
  std::shared_ptr<AbstractMotor> drive_bl = x_model->getBottomLeftMotor();
  std::shared_ptr<AbstractMotor> drive_br = x_model->getBottomRightMotor();
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

    // DriveToPosition::update();

    double forward = DriveToPosition::forward + master.get_analog(ANALOG_RIGHT_Y) * 0.787401574803;
    double strafe  = DriveToPosition::strafe  + master.get_analog(ANALOG_RIGHT_X) * 0.787401574803;
    // double turn    = DriveToPosition::turn    + master.get_analog(ANALOG_LEFT_X) * 0.787401574803 * ((master.get_analog(ANALOG_LEFT_Y) * 0.787401574803) / 100 + 1.1);
    double temp_turn    = master.get_analog(ANALOG_LEFT_X) * 0.787401574803;
    double turn    = DriveToPosition::turn    + pow(abs(temp_turn / 100), 1.8) * 100 * sgn(temp_turn);
    double m = std::min(1.0, 100 / (fabs(forward) + fabs(strafe) + fabs(turn)));

    drive_fl->moveVelocity((forward + strafe + turn) * 2 * m);
    drive_fr->moveVelocity((forward - strafe - turn) * 2 * m);
    drive_bl->moveVelocity((forward - strafe + turn) * 2 * m);
    drive_br->moveVelocity((forward + strafe - turn) * 2 * m);
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

controllerbuttons::Macro drive_test(
    [](){
      // while (true) {
      //   driveToPosition(0_in, 0_in, 0_deg);
      //   controllerbuttons::wait(5);
      // }
      DriveToPosition::addPositionTarget(0_in, 0_in, 0_deg, 0_in);
      DriveToPosition::addPositionTarget(20_in, 0_in, 0_deg, 0_in);
      DriveToPosition::addPositionTarget(30_in, 0_in, 0_deg, 0_in);
      DriveToPosition::addPositionTarget(40_in, 0_in, 0_deg, 0_in);
      controllerbuttons::wait(3000);
      DriveToPosition::addPositionTarget(0_in, 0_in, 0_deg, 0_in);
    }, 
    [](){
      // set_drive.forward = 0;
      // set_drive.strafe = 0;
      // set_drive.turn = 0;
    },
    {&test_group});

// Test function that prints to the terminal.
void single_use_button() {
  printf("single_use_button\n");
}







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


bool intakes_retracted = false;
bool intake_sensors_last = false;

bool intakes_extended() {
  bool intake_sensors = right_intake_sensor.get_value() < 20;
  bool result = !intakes_retracted || (intake_sensors_last && !intake_sensors && intake_right.get_target_velocity() > 10);
  intake_sensors_last = intake_sensors;
  intakes_retracted = !result;
  return result; 
}


void intake_back(pros::Motor &motor, pros::ADIAnalogIn &sensor) {
  motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  double start_pos = motor.get_position();
  int start_time = pros::millis();
  while (sensor.get_value() > 20 && pros::millis() - start_time < 600) {
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



void rollers_forward() {
  // bottom_roller.move(127);
  top_roller.move(127);
}

void rollers_reverse() {
  // bottom_roller.move(-127);
  top_roller.move(-127);
}

void rollers_stop() {
  // bottom_roller.move(0);
  top_roller.move(0);
}

namespace rollers {
  int score_queue = 0;
  int intake_queue = 0;
  int balls_in_robot = 0;
  double roller_pos_when_switch_pressed = 0;
  double ball_step = 600;
  // bool intake_toggle = false;
  bool ball_sensor_last = false;
  int t = 0;

  void main_task() {
    intake_left.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    intake_right.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    bottom_roller.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    top_roller.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    pros::delay(500);
    while (true) {
      bool ball_sensor_triggered = intakes_extended() && !ball_sensor_last && ball_sensor.get_value() < 2000;
      controllermenu::controller_print_array[0] = "BIR: " + std::to_string(balls_in_robot);
      // controllermenu::controller_print_array[0] = "LIS: " + std::to_string(left_intake_sensor.get_value());
      // controllermenu::controller_print_array[1] = "RIS: " + std::to_string(right_intake_sensor.get_value());
      controllermenu::controller_print_array[1] = "IR: " + std::to_string(intakes_retracted);
      controllermenu::controller_print_array[2] = "LS: " + std::to_string(ball_sensor.get_value());
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
}

controllerbuttons::Macro intakes_back(
    [](){
      intakes_retracted = true;
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
  button_handler.master.r2.pressed.set(intake_toggle);
  button_handler.master.l2.pressed.set_macro(intakes_back);
  button_handler.master.r1.pressed.set(rollers::test_intake);
  button_handler.master.l1.pressed.set(rollers::score_ball);
  button_handler.master.down.pressed.set(rollers_reverse);
  button_handler.master.down.released.set(rollers_stop);
  button_handler.master.up.pressed.set(rollers_forward);
  button_handler.master.up.released.set(rollers_stop);
}

} // namespace robotfunctions
