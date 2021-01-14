#include "main.h"

#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"
#include "auton-drive.h"
#include "auton-from-sd.h"
#include "ball-system.h"

// ChassisController chassis;

std::shared_ptr<OdomChassisController> chassis;
std::shared_ptr<ThreeEncoderXDriveModel> x_model;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

bool menu_enabled = true;

void set_drive_callbacks() {
  menu_enabled = false;
  controllerbuttons::button_handler.clear_group("menu");
  robotfunctions::set_callbacks();
  autondrive::set_callbacks();
  // controllermenu::master_print_array[0] = "";
  // controllermenu::master_print_array[1] = "";
  // controllermenu::master_print_array[2] = "";
}

void initialize() {
  tracker_left.reset_position();
  tracker_right.reset_position();
  tracker_back.reset_position();
  chassis = ChassisControllerBuilder()
    .withMotors(
        15,  // Top left
        -16, // Top right (reversed)
        -18, // Bottom right (reversed)
        17   // Bottom left
    )
    .withDimensions(AbstractMotor::gearset::green, {{3.25_in, 11.381_in}, imev5GreenTPR})
    .withSensors(
        RotationSensor{11, true}, // left encoder
        RotationSensor{13},  // right encoder
        RotationSensor{12}  // middle encoder
    )
    .withOdometry({{2.75_in, 12.0873_in, 6.04365_in, 2.75_in}, quadEncoderTPR}, StateMode::FRAME_TRANSFORMATION)
    .buildOdometry();
  chassis->getModel()->setBrakeMode(okapi::AbstractMotor::brakeMode::brake);
  x_model = std::dynamic_pointer_cast<ThreeEncoderXDriveModel>(chassis->getModel());
  // x_model->setBrakeMode(AbstractMotor::brakeMode::hold);
  pros::Task(autondrive::motor_task);
  // robotfunctions::set_callbacks();
  // ballsystem::set_callbacks();
  // ballsystem::init();
  // autondrive::set_callbacks();
  autonfromsd::load_autons_from_SD();
  controllermenu::init();
  pros::Task roller_task (robotfunctions::rollers::main_task);
  controllerbuttons::button_handler.master.r2.pressed.set(set_drive_callbacks);
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
void autonomous() {
  controllermenu::run_auton();

  // while (true) {
  //   pros::delay(150);
  // }
}
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
void opcontrol() {
  if (pros::competition::is_connected()) {
    set_drive_callbacks();
  }
  printf("opcontrol\n");
  Controller controller;
  while (true) {
    controllerbuttons::run_buttons();
    // ballsystem::debug();
    if (!menu_enabled) {
      QLength x = chassis->getState().x;
      QLength y = chassis->getState().y;
      QAngle theta = chassis->getState().theta;
      controllermenu::master_print_array[0] = std::to_string(tracker_left.get_position())  + " " + std::to_string(x.convert(inch));
      controllermenu::master_print_array[1] = std::to_string(tracker_right.get_position()) + " " + std::to_string(y.convert(inch)));
      controllermenu::master_print_array[2] = std::to_string(tracker_back.get_position())  + " " + std::to_string(theta.convert(degree));
    }
    pros::delay(10);
  }
}
