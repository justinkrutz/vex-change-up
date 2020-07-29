#include "api.h"

#include "robot-config.h"
#include "controller-buttons.h"
#include "robot-functions.h"


namespace robotfunctions {
controllerbuttons::MacroGroup test1;
controllerbuttons::MacroGroup test2;
controllerbuttons::MacroGroup test3;
controllerbuttons::MacroGroup abort;

void count_up_task() {
  printf("start\n");
  int count = 0;

  for (int i = 0; i < 50; i++) {
    printf("Up %d\n", count ++);
    pros::delay(20);
  }
}

void count_down_task() {
  printf("start\n");
  int count = 0;

  for (int i = 0; i < 50; i++) {
    printf("Down %d\n", count --);
    pros::delay(20);
  }
}

void count_task(void * arg) {
 printf("start\n");
 int count = 0;

 for (int i = 0; i < *(int*) arg; i++) {
   printf("Count %d\n", count ++);
   pros::delay(20);
 }
}

// Test function that prints to the terminal.
void single_use_button() {
  printf("single_use_button\n");
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
void abort_test1() {
  controllerbuttons::interrupt_macro_group(&test1);
}

void abort_test2() {
  controllerbuttons::interrupt_macro_group(&test2);
}

void count_up_task_hold_abort() {
  controllerbuttons::interrupt_macro_group(&test3);
}

void set_callbacks() {
  using namespace controllerbuttons;
  macro_group_vector = {&test1, &test2, &test3, &abort};
  button_callbacks = {
      {&master, BTN_A,     false, {&test1, &test2}, &count_down_task},
      {&master, BTN_Y,     false, {&test1},         &count_up_task},
      {&master, BTN_X,     false, {&test1},         &single_use_button},
      {&master, BTN_RIGHT, false, {&test2},         &count_down_task},
      {&master, BTN_LEFT,  false, {&test2, &test3}, &count_up_task},
      {&master, BTN_LEFT,   true, {&abort},         &count_up_task_hold_abort},
      {&master, BTN_UP,     true, {&test2},         &single_use_button},
      {&master, BTN_B,     false, {&abort},         &abort_test1},
      {&master, BTN_DOWN,  false, {&abort},         &abort_test2},
  };
}

} // namespace robotfunctions
