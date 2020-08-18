#include "api.h"

pros::Controller master (pros::E_CONTROLLER_MASTER);
pros::Controller partner (pros::E_CONTROLLER_PARTNER);
// pros::Motor drive_fl (15, pros::E_MOTOR_GEARSET_18, false);
// pros::Motor drive_fr (16, pros::E_MOTOR_GEARSET_18, true);
// pros::Motor drive_bl (17, pros::E_MOTOR_GEARSET_18, false);
// pros::Motor drive_br (18, pros::E_MOTOR_GEARSET_18, true);
pros::Motor test_motor_1 (10, pros::E_MOTOR_GEARSET_06, false);
pros::Motor test_motor_2 (20, pros::E_MOTOR_GEARSET_06, true);
pros::Motor test_motor_3 (9, pros::E_MOTOR_GEARSET_06, false);
pros::ADIEncoder tracker_left ('A', 'B', false);
pros::ADIEncoder tracker_back ('C', 'D', false);
pros::ADIEncoder tracker_right ('E', 'F', false);
// pros::ADIEncoder sensor (1, 2, false);
pros::Vision vision_sensor (7);
pros::Imu imu_sensor (6);
