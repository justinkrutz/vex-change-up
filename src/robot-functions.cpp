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

void motorTask()
{
  std::shared_ptr<AbstractMotor> drive_fl = x_model->getTopLeftMotor();
  std::shared_ptr<AbstractMotor> drive_fr = x_model->getTopRightMotor();
  std::shared_ptr<AbstractMotor> drive_bl = x_model->getBottomLeftMotor();
  std::shared_ptr<AbstractMotor> drive_br = x_model->getBottomRightMotor();
  double forward;
  double strafe;
  double turn;
  double m;
  while(1)
  {
  double forward = SetDrive.forward + master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
  double strafe  = SetDrive.strafe  + master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
  double turn    = SetDrive.turn    + master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

  if(fabs(forward) + fabs(strafe) + fabs(turn) > 100) {
    m = 100 / (fabs(forward) + fabs(strafe) + fabs(turn));
    forward = forward * m;
    strafe  = strafe  * m;
    turn    = turn    * m;
  }
  drive_fl->moveVelocity((forward + strafe + turn) * 0.5);
  drive_fr->moveVelocity((forward - strafe - turn) * 0.5);
  drive_bl->moveVelocity((forward - strafe + turn) * 0.5);
  drive_br->moveVelocity((forward + strafe - turn) * 0.5);
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
        SetDrive.forward = cos(chassis->getState().theta.convert(radian));
        SetDrive.strafe = -sin(chassis->getState().theta.convert(radian));
        SetDrive.turn = 1;
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
