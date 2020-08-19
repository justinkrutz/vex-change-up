#include "main.h"

#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"
#include "autonomous.h"


namespace robotfunctions {
controllerbuttons::MacroGroup test_group;

template <typename T> int sgn(T &&val) {
  if (val < 0) {
    return -1;
  } else {
    return 1;
  }
}

struct rampMathSettings {
  int start_output;
  int max_output;
  int end_output;
  double ramp_up_p;
  double ramp_down_p;
};

int rampMath(double input, double total_range, rampMathSettings s) {
  int output;
  double ramp_up_range = ((s.max_output - s.start_output)*s.ramp_up_p);
  double ramp_down_range = ((s.max_output - s.end_output)*s.ramp_down_p);
  double ramp_up_muliplier = ((s.max_output - s.start_output) / ramp_up_range);
  double ramp_down_muliplier = ((s.max_output - s.end_output) / ramp_down_range);
      if (fabs(input) < fabs(ramp_up_range)) {
        output = ((input * ramp_up_muliplier) + s.start_output);
      } else if (fabs(input) >= (fabs(total_range) - fabs(ramp_down_range))) {
        output = ((total_range - input) * ramp_down_muliplier + s.end_output);
      } else {
        output = (s.max_output);
      }
  return output;
}

bool targetPositionEnabled = false;

struct Position {
  QLength x = 0_in;
  QLength y = 0_in;
  QAngle theta = 0_deg;
  QLength offset = 0_in;
  // robotfunctions::Position starting_pos;
};

OdomState starting_position;

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

void setPositionTarget(QLength x, QLength y, QAngle theta, QLength offset) {
  targets.push({x, y, theta, offset});
  targetPositionEnabled = true;
}



void driveToPosition(Target target, rampMathSettings move_settings, rampMathSettings turn_settings) {
  Point point{target.end.x, target.end.y};
  auto [magnitude, direction] = OdomMath::computeDistanceAndAngleToPoint(point, chassis->getState());
  OdomState start_state {target.start.x, target.start.y, target.start.theta};
  auto [start_magnitude, start_direction] = OdomMath::computeDistanceAndAngleToPoint(point, start_state);
  double move_speed = rampMath(magnitude.convert(inch), start_magnitude.convert(inch), move_settings);
  double turn_speed = rampMath(direction.convert(radian), start_direction.convert(radian), turn_settings);
  set_drive.forward = move_speed * cos(direction.convert(radian));
  set_drive.strafe  = move_speed * sin(direction.convert(radian));
  set_drive.turn    = turn_speed;
  controllermenu::controller_print_array[0] = "dir: " + std::to_string(direction.convert(degree));
  controllermenu::controller_print_array[1] = "mag: " + std::to_string(magnitude.convert(inch));
}





void motorTask()
{
  std::shared_ptr<AbstractMotor> drive_fl = x_model->getTopLeftMotor();
  std::shared_ptr<AbstractMotor> drive_fr = x_model->getTopRightMotor();
  std::shared_ptr<AbstractMotor> drive_bl = x_model->getBottomLeftMotor();
  std::shared_ptr<AbstractMotor> drive_br = x_model->getBottomRightMotor();
  while(1)
  {

  if (targetPositionEnabled) {
    if (targets.size() == 0) {
      set_drive.forward = 0;
      set_drive.strafe = 0;
      set_drive.turn = 0;
    } else if (targets.size() == 1) {
      driveToPosition(targets.front(),
                      {100, 100, 20, 8, 8},
                      {100, 100, 20, 100, 100});
    } else {
      driveToPosition(targets.front(),
                      {100, 100, 100, 8, 8},
                      {100, 100, 100, 100, 100});
      
      OdomState state_target{targets.front().end.x, targets.front().end.y, targets.front().end.theta};
      Point starting_point{targets.front().end.x, targets.front().end.y};
      auto [magnitude_target, direction_target] = OdomMath::computeDistanceAndAngleToPoint(starting_point, state_target);
      auto [magnitude_real, direction_real] = OdomMath::computeDistanceAndAngleToPoint(starting_point, chassis->getState());
      if (magnitude_real + 2_in >= magnitude_target && direction_real + 10_deg >= direction_target) {
        targets.pop();
      }
      // slow down and hold position
    }
  }

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

  double forward = set_drive.forward + master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
  double strafe  = set_drive.strafe  + master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
  double turn    = set_drive.turn    + master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

  if(fabs(forward) + fabs(strafe) + fabs(turn) > 100) {
    double m = 100 / (fabs(forward) + fabs(strafe) + fabs(turn));
    forward = forward * m;
    strafe  = strafe  * m;
    turn    = turn    * m;
  }
  drive_fl->moveVelocity((forward + strafe + turn) * 2);
  drive_fr->moveVelocity((forward - strafe - turn) * 2);
  drive_bl->moveVelocity((forward - strafe + turn) * 2);
  drive_br->moveVelocity((forward + strafe - turn) * 2);
  pros::delay(5);
  }
}


void intakeBalls(int balls) {
}

void scoreBalls(int balls) {
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
    {&test_group});

controllerbuttons::Macro drive_test(
    [](){
      // while (true) {
      //   driveToPosition(0_in, 0_in, 0_deg);
      //   controllerbuttons::wait(5);
      // }
      setPositionTarget(10 * targets.size() * inch, 0_in, 0_deg, 0_in);
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

/*===========================================================================*/

void set_callbacks() {
  using namespace controllerbuttons;
  button_handler.master.a.pressed.set_macro(drive_test);
  button_handler.master.a.released.set([&](){ drive_test.terminate(); });
  // button_handler.master.a.pressed.set_macro(count_up);
  // button_handler.master.a.released.set([&](){ count_up.terminate(); });
  // button_handler.master.x.pressed.set([&](){ test_group.terminate(); });
  // button_handler.master.left.pressed.set(single_use_button, {}, {&test_group});
  // button_handler.master.a.pressed.set([](){ driveToClosestGoal(); }, {}, {&test_group});

  // button_handler.master.r2.pressed.set([](){ test_motor_1.move(127); }, {}, {&test_group});
  // button_handler.master.r1.pressed.set([](){ test_motor_1.move(-127); }, {}, {&test_group});
  // button_handler.master.l2.pressed.set([](){ test_motor_2.move(127); }, {}, {&test_group});
  // button_handler.master.l1.pressed.set([](){ test_motor_2.move(-127); }, {}, {&test_group});
  // button_handler.master.right.pressed.set([](){ test_motor_3.move(-127); }, {}, {&test_group});
  // button_handler.master.left.pressed.set([](){
  //     test_motor_3.move(0);
  //     test_motor_3.move_relative(100, 127);
  //   }, {}, {&test_group});
  // button_handler.master.b.pressed.set([](){
  //     test_motor_3.move(0);
  //     test_motor_2.move(0);
  //     test_motor_1.move(0);
  //   }, {}, {&test_group});
}

} // namespace robotfunctions
