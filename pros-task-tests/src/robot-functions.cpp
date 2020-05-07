#include "api.h"

#include "robot-config.h"
#include "controller-buttons.h"
#include "robot-functions.h"


namespace robotfunctions {
controllerbuttons::MacroGroup test1;
controllerbuttons::MacroGroup test2;
controllerbuttons::MacroGroup test3;
controllerbuttons::MacroGroup abort;

void countUpTask() {
  printf("start\n");
  int count = 0;

  for (int i = 0; i < 50; i++) {
    printf("Up %d\n", count ++);
    pros::delay(20);
  }
}

void countDownTask() {
  printf("start\n");
  int count = 0;

  for (int i = 0; i < 50; i++) {
    printf("Down %d\n", count --);
    pros::delay(20);
  }
}

void countTask(void * arg) {
 printf("start\n");
 int count = 0;

 for (int i = 0; i < *(int*) arg; i++) {
   printf("Count %d\n", count ++);
   pros::delay(20);
 }
}

// Test function that prints to the terminal.
void singleUseButton() {
  printf("singleUseButton\n");
}

// Dectect and cycle through warnings on the controller screen.
void checkForWarnings() {
  while (1) {
    // WARN_BOOL(TestMotor1, installed());
    // WARN_BOOL(TestMotor2, installed());
    // WARN_BOOL(Vision, installed());
    // WARN_BOOL(Inertial, installed());
    // WARN_BOOL(Controller2, installed());
    // WARN_BOOL(Brain.SDcard, isInserted());
    // WARN_RANGE(Inertial, rotation(), -2, 2);

    pros::delay(10);
  }
}

/*===========================================================================*/

// Abort the test
void abortTest1() {
  controllerbuttons::interruptMacroGroup(&test1);
}

void abortTest2() {
  controllerbuttons::interruptMacroGroup(&test2);
}

void countUpTaskHoldAbort() {
  controllerbuttons::interruptMacroGroup(&test3);
}
void setCallbacks() {
  using namespace controllerbuttons;
  using namespace pros;
  macro_group_vector = {&test1, &test2, &test3, &abort};
  button_callbacks = {
      {&master, E_CONTROLLER_DIGITAL_A,     false, {&test1,  &test2},  &countDownTask},
      {&master, E_CONTROLLER_DIGITAL_Y,     false, {&test1}, &countUpTask},
      {&master, E_CONTROLLER_DIGITAL_X,     false, {&test1}, &singleUseButton},
      {&master, E_CONTROLLER_DIGITAL_RIGHT, false, {&test2}, &countDownTask},
      {&master, E_CONTROLLER_DIGITAL_LEFT,  false, {&test2, &test3}, &countUpTask},
      {&master, E_CONTROLLER_DIGITAL_LEFT,   true, {&abort}, &countUpTaskHoldAbort},
      {&master, E_CONTROLLER_DIGITAL_UP,    false, {&test2}, &singleUseButton},
      {&master, E_CONTROLLER_DIGITAL_B,     false, {&abort}, &abortTest1},
      {&master, E_CONTROLLER_DIGITAL_DOWN,  false, {&abort}, &abortTest2},
  };
}

} // namespace robotfunctions
