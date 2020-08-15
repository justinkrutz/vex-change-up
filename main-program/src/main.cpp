#include "main.h"

#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"
#include "field-position.h"

// ChassisController chassis;

std::shared_ptr<OdomChassisController> chassis;

void initialize_task() {
  // controllermenu::set_callbacks();
  controllermenu::init();
  // controllermenu::load_settings();
  // (pros::Task(robotfunctions::check_for_warnings));
  // waitUntil(pros::competition::is_connected());
  // master.clear();
  // printf("print %d\r\n", master.print(1, 1, "Connected %d", 1));
  // master.print(1, 1, "Connected %d", 1);
  // controllermenu::check_for_auton();
  // robotfunctions::set_callbacks();
  // robotfunctions::motorTask();
  // controllerbuttons::set_callbacks();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  chassis = ChassisControllerBuilder()
    .withMotors(
        15,  // Top left
        -16, // Top right (reversed)
        -18, // Bottom right (reversed)
        17   // Bottom left
    )
    .withDimensions(AbstractMotor::gearset::green, {{3.25_in, 11.381_in}, imev5GreenTPR})
    .withSensors(
        // ADIEncoder{'E', 'F'}, // left encoder in ADI ports A & B
        // ADIEncoder{'A', 'B', true},  // right encoder in ADI ports C & D (reversed)
        // ADIEncoder{'C', 'D'}  // middle encoder in ADI ports E & F
        ADIEncoder{'E', 'F'}, // left encoder in ADI ports A & B
        ADIEncoder{'A', 'B'},  // right encoder in ADI ports C & D (reversed)
        ADIEncoder{'C', 'D'}  // middle encoder in ADI ports E & F
    )
    .withOdometry({{2.75_in, 11.3_in, 4.8_in, 2.75_in}, quadEncoderTPR}, StateMode::CARTESIAN)
    .buildOdometry();

// drive = ChassisControllerBuilder()
//         .withMotors(1, -10)
//         // Green gearset, 4 in wheel diam, 11.5 in wheel track
//         .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
//         .build();

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
void opcontrol() {
  jsonTest();
  printf("opcontrol\n");
  // set the state to zero
  chassis->setState({75_in, 110_in, 0_deg});
  // chassis->setState({0_in, 0_in, 0_deg});
  // chassis->driveToPoint({1_ft, 0_ft});
  // chassis->moveDistance(12_in);
  // turn 45 degrees and drive approximately 1.4 ft
  // chassis->driveToPoint({1_ft, 1_ft});
  // turn approximately 45 degrees to end up at 90 degrees
  // chassis->turnToAngle(90_deg);
  // turn approximately -90 degrees to face {5_ft, 0_ft} which is to the north of the robot
  // chassis->turnToPoint({5_ft, 0_ft});

  Controller controller;
  auto x_model = std::dynamic_pointer_cast<ThreeEncoderXDriveModel>(chassis->getModel());
  while (true) {
    controllerbuttons::run_buttons();

    // QLength x = chassis->getState().x;
    // QLength y = chassis->getState().y;
    // QAngle theta = chassis->getState().theta;
    // controllermenu::controller_print_array[0] = std::to_string(x.convert(inch));
    // controllermenu::controller_print_array[1] = std::to_string(y.convert(inch));
    // controllermenu::controller_print_array[2] = std::to_string(theta.convert(degree));
    
    
    // printf("tracker_left: %d tracker_right: %d tracker_back: %d\n", tracker_left.get_value(), tracker_right.get_value(), tracker_back.get_value());
    // x_model->xArcade(controller.getAnalog(ControllerAnalog::rightX),
    //                 controller.getAnalog(ControllerAnalog::rightY),
    //                 controller.getAnalog(ControllerAnalog::leftX));
    pros::delay(10);
  }
}