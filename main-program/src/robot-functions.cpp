#include "api.h"

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
  double forward;
  double strafe;
  double turn;
  double m;
  while(1)
  {
  double forward = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
  double strafe = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
  double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

  if(fabs(forward) + fabs(strafe) + fabs(turn) > 127) {
    m = 127 / (fabs(forward) + fabs(strafe) + fabs(turn));
    forward = forward * m;
    strafe  = strafe  * m;
    turn    = turn    * m;
  }

  // drive_fl.move(forward + strafe + turn);
  // drive_fr.move(forward - strafe - turn);
  // drive_bl.move(forward - strafe + turn);
  // drive_br.move(forward + strafe - turn);
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
    {&test_group});

// Test function that prints to the terminal.
void single_use_button() {
  printf("single_use_button\n");
}

/*===========================================================================*/

void set_callbacks() {
  using namespace controllerbuttons;
  // button_handler.master.a.pressed.set_macro(count_up);
  // button_handler.master.a.released.set([&](){ count_up.terminate(); });
  // button_handler.master.x.pressed.set([&](){ test_group.terminate(); });
  // button_handler.master.left.pressed.set(single_use_button, {}, {&test_group});
  button_handler.master.a.pressed.set([](){ driveToClosestGoal(); }, {}, {&test_group});

  button_handler.master.r2.pressed.set([](){ test_motor_1.move(127); }, {}, {&test_group});
  button_handler.master.r1.pressed.set([](){ test_motor_1.move(-127); }, {}, {&test_group});
  button_handler.master.l2.pressed.set([](){ test_motor_2.move(127); }, {}, {&test_group});
  button_handler.master.l1.pressed.set([](){ test_motor_2.move(-127); }, {}, {&test_group});
  button_handler.master.right.pressed.set([](){ test_motor_3.move(-127); }, {}, {&test_group});
  button_handler.master.left.pressed.set([](){
      test_motor_3.move(0);
      test_motor_3.move_relative(100, 127);
    }, {}, {&test_group});
  button_handler.master.b.pressed.set([](){
      test_motor_3.move(0);
      test_motor_2.move(0);
      test_motor_1.move(0);
    }, {}, {&test_group});
}

} // namespace robotfunctions
