#include "main.h"


std::shared_ptr<OdomChassisController> chassis;
std::shared_ptr<ThreeEncoderXDriveModel> x_model;

pros::Controller master (pros::E_CONTROLLER_MASTER);
pros::Controller partner (pros::E_CONTROLLER_PARTNER);
// okapi::Motor drive_fl (9, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
// okapi::Motor drive_fr (10, true,  AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
// okapi::Motor drive_bl (1, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
// okapi::Motor drive_br (2, true,  AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
pros::Motor intake_left (3, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor intake_right (8, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor top_roller (12, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor bottom_roller (20, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);

pros::Imu imu (4);
pros::Rotation tracker_left (5);
pros::Rotation tracker_back (6);
pros::Rotation tracker_right (7);

pros::Distance distance_sensor_right(15);
pros::Distance distance_sensor_left(16);
pros::Optical optical_sensor(17);

pros::ADILineSensor ball_sensor_intake ('A');
pros::ADILineSensor ball_sensor_bottom ('B');
pros::ADILineSensor ball_sensor_middle ('C');
pros::ADILineSensor ball_sensor_top    ('D');
pros::ADILineSensor ball_sensor_score  ('E');
//                                     ('F');
pros::ADILineSensor goal_sensor_one    ('G');
pros::ADILineSensor goal_sensor_two    ('H');

void build_chassis() {
  tracker_left.reset_position();
  tracker_right.reset_position();
  tracker_back.reset_position();

  chassis = ChassisControllerBuilder()
    .withMotors(
        9,  // Top left
        -10, // Top right (reversed)
        -2, // Bottom right (reversed)
        1   // Bottom left
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
}