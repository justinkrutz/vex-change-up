#include "api.h"

pros::Controller master (pros::E_CONTROLLER_MASTER);
pros::Controller partner (pros::E_CONTROLLER_PARTNER);
// pros::Motor drive_fl (15, pros::E_MOTOR_GEARSET_18, false);
// pros::Motor drive_fr (16, pros::E_MOTOR_GEARSET_18, true);
// pros::Motor drive_bl (17, pros::E_MOTOR_GEARSET_18, false);
// pros::Motor drive_br (18, pros::E_MOTOR_GEARSET_18, true);
pros::Motor intake_left (10, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor intake_right (20, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor bottom_roller (9, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor top_roller (8, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::ADIEncoder tracker_right ('A', 'B', false);
pros::ADIEncoder tracker_left ('C', 'D', false);
pros::ADIEncoder tracker_back ('E', 'F', false);
pros::ADIDigitalIn top_ball_sensor ('G');
// pros::ADIAnalogIn left_intake_sensor ('G');
// pros::ADIAnalogIn right_intake_sensor ('H');
pros::ADILineSensor bottom_ball_sensor ('H');
// pros::Imu imu_sensor (6);