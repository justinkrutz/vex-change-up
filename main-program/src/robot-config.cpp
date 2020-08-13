#include "api.h"

pros::Controller master (pros::E_CONTROLLER_MASTER);
pros::ADIEncoder sensor ('A', 'B', false);
pros::Controller partner (pros::E_CONTROLLER_PARTNER);
pros::Motor fl_drive (15, pros::E_MOTOR_GEARSET_18, false);
pros::Motor fr_drive (16, pros::E_MOTOR_GEARSET_18, true);
pros::Motor bl_drive (17, pros::E_MOTOR_GEARSET_18, false);
pros::Motor br_drive (18, pros::E_MOTOR_GEARSET_18, true);
pros::Motor test_motor_1 (10, pros::E_MOTOR_GEARSET_06, false);
pros::Motor test_motor_2 (20, pros::E_MOTOR_GEARSET_06, true);
pros::Motor test_motor_3 (9, pros::E_MOTOR_GEARSET_06, false);
pros::Vision vision_sensor (7);
pros::Imu imu_sensor (6);
