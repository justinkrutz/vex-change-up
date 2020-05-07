#include "main.h"

#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"

void initialize_task() {
  // controllermenu::printMenu();
  // controllermenu::loadSettings();
  // controllermenu::setCallbacks();
  // (thread(robotfunctions::checkForWarnings));
  // (thread (printGraphData));
  // waitUntil(Competition.isCompetitionSwitch() || Competition.isFieldControl());
  // Controller1.Screen.clearScreen();
  // Controller1.Screen.setCursor(1, 0);
  // Controller1.Screen.print("Connected");
  // controllermenu::checkForAuton();
  robotfunctions::setCallbacks();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  (pros::Task (initialize_task));
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */


void foo_one() {
  printf("foo_one\r\n");
  pros::delay(100);
}

void foo_two() {
  printf("foo_two\r\n");
  pros::delay(100);
}

void foo_three() {
  printf("foo_three\r\n");
  pros::delay(100);
}

int x;

int *x_ptr = &x;

void foo(void * arg) {
  int i = (*(int*) arg)++;
  printf("foo %d\n", i);
  pros::delay(100);
}

void stop_task(pros::task_t task) {
  if (pros::c::task_get_state(task) != pros::E_TASK_STATE_DELETED &&
      pros::c::task_get_state(task) != pros::E_TASK_STATE_INVALID) {
    pros::c::task_delete(task);
  }
}

pros::task_t task_one_t;

void opcontrol() {

  // pros::Task task_one (foo_one);
  // printf("foo state: %d\r\n", pros::c::task_get_state(task_one_t));
  stop_task(task_one_t);
  task_one_t = pros::c::task_create (foo, x_ptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "task_one");
  pros::delay(5);
  printf("foo state: %d\r\n", pros::c::task_get_state(task_one_t));
  stop_task(task_one_t);
  pros::delay(5);
  printf("foo state: %d\r\n", pros::c::task_get_state(task_one_t));
  stop_task(task_one_t);



  // (pros::Task(foo, x_ptr, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "My Task"));
  // pros::delay(20);
  // printf("foo state: %d\r\n", pros::Task(foo).get_state());
  // pros::delay(20);
  // (pros::Task(foo)).remove();

	while (true) {
    // controllerbuttons::runButtons();
		pros::delay(10);
	}
}
