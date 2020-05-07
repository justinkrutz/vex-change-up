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

int i;

pros::task_t my_task;

 void my_task_fn(void* ign) {
     // while(pros::c::task_notify_take(true, TIMEOUT_MAX)) {
     //     printf("%d\r\n", i++);
     // }
     printf("start\r\n");
     pros::c::task_notify(my_task);
     pros::delay(3000);
     pros::c::task_notify_take(true, TIMEOUT_MAX);
     printf("end\r\n");
 }

void opcontrol() {
  // my_task = pros::c::task_create(my_task_fn, NULL, TASK_PRIORITY_DEFAULT,
  //                                 TASK_STACK_DEPTH_DEFAULT, "Notify me! Task");
	while (true) {
    // if(master.get_digital(DIGITAL_X)) {
    // pros::c::task_notify_clear(my_task);
    // }
    controllerbuttons::run_buttons();
		pros::delay(10);
	}
}
