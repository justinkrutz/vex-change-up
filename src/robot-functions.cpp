#include "main.h"

#include "robot-config.h"
#include "controller-buttons.h"
#include "robot-functions.h"
#include "autonomous.h"


namespace robotfunctions {
// controllerbuttons::MacroGroup test1;
// controllerbuttons::MacroGroup test2;
// controllerbuttons::MacroGroup test3;
// controllerbuttons::MacroGroup abort;

controllerbuttons::MacroGroup test_group;

template <typename T> int sgn(T val) {
  if (val < 0) {
    return -1;
  } else {
    return 1;
  }
}

void motorTask()
{
  std::shared_ptr<AbstractMotor> drive_fl = x_model->getTopLeftMotor();
  std::shared_ptr<AbstractMotor> drive_fr = x_model->getTopRightMotor();
  std::shared_ptr<AbstractMotor> drive_bl = x_model->getBottomLeftMotor();
  std::shared_ptr<AbstractMotor> drive_br = x_model->getBottomRightMotor();
  while(1)
  {
  double ctr_f = master.get_analog(ANALOG_RIGHT_Y) * 0.787401574803;
  double ctr_s = -master.get_analog(ANALOG_RIGHT_X) * 0.787401574803;
  double ctr_t = master.get_analog(ANALOG_LEFT_X) * 0.787401574803;
  double theta = 0;
  if (ctr_f != 0) {
    theta = atan(ctr_s / ctr_f);
  } else {
    theta = 90 * degreeToRadian * sgn(ctr_s);
  }
  double move_m = sqrt(pow(ctr_f, 2) + pow(ctr_s, 2)) * sgn(ctr_f);
  double forward = move_m * cos(theta + chassis->getState().theta.convert(radian));
  double strafe  = move_m * -sin(theta + chassis->getState().theta.convert(radian));
  double turn    = ctr_t;

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
  chassis->driveToPoint({x, y}, false, offset);
  // chassis->turnToAngle({theta});
  // SetDrive.forward = cos(chassis->getState().theta.convert(radian));
  // SetDrive.strafe = sin(chassis->getState().theta.convert(radian));
  // SetDrive.turn = 1;
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
        SetDrive.forward = cos(chassis->getState().theta.convert(radian)) * 100;
        SetDrive.strafe = -sin(chassis->getState().theta.convert(radian)) * 100;
        SetDrive.turn = 100;
        controllerbuttons::wait(5);
      }
    }, 
    [](){
      SetDrive.forward = 0;
      SetDrive.strafe = 0;
      SetDrive.turn = 0;
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
