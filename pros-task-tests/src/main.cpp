#include "main.h"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
// void on_center_button() {
// 	static bool pressed = false;
// 	pressed = !pressed;
// 	if (pressed) {
// 		pros::lcd::set_text(2, "I was pressed!");
// 	} else {
// 		pros::lcd::clear_line(2);
// 	}
// }

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {}

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

 int x = 50;

int *x_ptr = &x;

void foo(void * arg) {
  int i = (*(int*) arg)++;
  printf("foo %d\n", i);
}

void my_task_fn(void* param) {
  printf("%d\n",*(char*)param);
}

void null_function() {

}
struct MyStruct {
	// pros::task_t taskt;
	pros::Task task = null_function;
};

std::vector<MyStruct> my_vector = {{}, {}};

MyStruct my_object;

pros::task_t t;



void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor left_mtr(1);
	pros::Motor right_mtr(2);

	// pros::Task task2(__null);
  printf("state1 %d\n", my_object.task.get_state());
  my_object.task = countDownTask;
	pros::delay(4000);
  printf("state2 %d\n", my_object.task.get_state());
	my_object.task = countUpTask;
  pros::delay(20);
  printf("state3 %d\n", my_object.task.get_state());

	while (true) {
		pros::delay(10);
	}
}
