#include "api.h"

pros::Controller master (pros::E_CONTROLLER_MASTER);
pros::Controller partner (pros::E_CONTROLLER_PARTNER);
pros::Motor test_motor_1 (10, pros::E_MOTOR_GEARSET_18, false);
pros::Motor test_motor_2 (20, pros::E_MOTOR_GEARSET_18, false);
pros::Vision vision_sensor (7);
pros::Imu imu_sensor (6);
