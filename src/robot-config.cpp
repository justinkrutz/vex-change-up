#include "api.h"

pros::Controller master (pros::E_CONTROLLER_MASTER);
pros::Controller partner (pros::E_CONTROLLER_PARTNER);
// pros::Motor drive_fl (15, pros::E_MOTOR_GEARSET_18, false);
// pros::Motor drive_fr (16, pros::E_MOTOR_GEARSET_18, true);
// pros::Motor drive_bl (17, pros::E_MOTOR_GEARSET_18, false);
// pros::Motor drive_br (18, pros::E_MOTOR_GEARSET_18, true);
pros::Motor intake_left (10, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor intake_right (20, pros::E_MOTOR_GEARSET_06, true, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor bottom_roller (9, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::Motor top_roller (8, pros::E_MOTOR_GEARSET_06, false, pros::E_MOTOR_ENCODER_DEGREES);
pros::ADIEncoder tracker_left ('A', 'B', false);
pros::ADIEncoder tracker_back ('C', 'D', false);
pros::ADIEncoder tracker_right ('E', 'F', false);
pros::ADIDigitalIn front_ball_limit_switch ('G');
pros::ADIDigitalIn back_ball_limit_switch ('H');
// pros::ADIEncoder sensor (1, 2, false);
pros::Vision vision_sensor (7);
pros::Imu imu_sensor (6);