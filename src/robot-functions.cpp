#include "main.h"

#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"
#include "autonomous.h"


namespace robotfunctions {
// controllerbuttons::MacroGroup test1;
// controllerbuttons::MacroGroup test2;
// controllerbuttons::MacroGroup test3;
// controllerbuttons::MacroGroup abort;

template <typename T> int sgn(T &&val) {
  if (val < 0) {
    return -1;
  } else {
    return 1;
  }
}

int rampMath(double input, double totalRange, int startOutput, int maxOutput, int endOutput, double rampUpP = 0.1, double rampDownP = 0.12) {
  int output;
  double rampUpRange = ((maxOutput - startOutput)*rampUpP);
  double rampDownRange = ((maxOutput - endOutput)*rampDownP);
  double rampUpMuliplier = ((maxOutput - startOutput) / rampUpRange);
  double rampDownMuliplier = ((maxOutput - endOutput) / rampDownRange);
      if (fabs(input) < fabs(rampUpRange)) {
        output = ((input * rampUpMuliplier) + startOutput);
      } else if (fabs(input) >= (fabs(totalRange) - fabs(rampDownRange))) {
        output = ((totalRange - input) * rampDownMuliplier + endOutput);
      } else {
        output = (maxOutput);
      }
  return output;
}

bool targetPositionEnabled = false;

struct position {
  QLength x;
  QLength y;
  QAngle theta;
  QLength offset;
};

std::vector<position> targets = {};

// namespace positiontarget{
//   QLength x;
//   QLength y;
//   QAngle theta;
//   QLength offset;
// } // positiontarget

void setPositionTarget(QLength x, QLength y, QAngle theta, QLength offset) {
  // positiontarget::x = x;
  // positiontarget::y = y;
  // positiontarget::theta = theta;
  // positiontarget::offset = offset;

  targetPositionEnabled = true;
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
    if (targets.size() > 1) {
      // slow down and hold position
    } else {
      // don't slow down
      // if () {
      //   targets.erase(targets.begin());
      // }
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

void driveToPosition(QLength x, QLength y, QAngle theta, QLength offset) {
  Point point{x, y};

  auto [magnitude, direction] = OdomMath::computeDistanceAndAngleToPoint(point, chassis->getState());
  // direction = 45_deg;
  direction;

  double move_speed = std::min(100.0, magnitude.convert(inch)*8);
  // double move_speed = 50;
  double turn_speed = 100 * (theta - chassis->getState().theta).convert(radian);
  // magnitude.convert(inch)
  controllermenu::controller_print_array[0] = "dir: " + std::to_string(direction.convert(degree));
  controllermenu::controller_print_array[1] = "mag: " + std::to_string(magnitude.convert(inch));
  // controllermenu::controller_print_array[1] = "y: " + y_str;
  set_drive.forward = move_speed * cos(direction.convert(radian));
  set_drive.strafe  = move_speed * sin(direction.convert(radian));
  set_drive.turn    = turn_speed;


  // double ctr_f;
  // double ctr_s;
  // double ctr_t;
  // chassis->driveToPoint({x, y}, false, offset);
  // chassis->turnToAngle({theta});
  // set_drive.forward = cos(chassis->getState().theta.convert(radian));
  // set_drive.strafe = sin(chassis->getState().theta.convert(radian));

  // double x_pos = chassis->getState().x.convert(inch);
  // double y_pos = chassis->getState().y.convert(inch);
  // double direction = 0;
  // if (ctr_f != 0) {
  //   direction = atan(ctr_s / ctr_f);
  // } else {
  //   direction = 90 * degreeToRadian * sgn(ctr_s);
  // }
  // double magnitude = sqrt(pow(ctr_f, 2) + pow(ctr_s, 2)) * sgn(ctr_f);

  // set_drive.turn = 100;
  // set_drive.forward = move_speed * cos(chassis->getState().theta.convert(radian));
  // set_drive.strafe = move_speed * -sin(chassis->getState().theta.convert(radian));
  // set_drive.turn = turn_speed;
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
      while (true) {
        driveToPosition(0_in, 0_in, 0_deg);
        controllerbuttons::wait(5);
      }
    }, 
    [](){
      set_drive.forward = 0;
      set_drive.strafe = 0;
      set_drive.turn = 0;
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
