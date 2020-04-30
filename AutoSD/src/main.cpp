/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Justin Krutz                                              */
/*    Created:      Thursday Apr 30 2020                                      */
/*    Description:  Record and playback autonomous from sd card               */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Motor10              motor         10              
// Motor20              motor         20              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

#include <cstdio>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>

#include <bits/stdc++.h>

struct AutonStruct {
  double step_time_ms;
  double step_pos_deg;
};

AutonStruct test_auton_motor10 {5, 10};
AutonStruct test_auton_motor20 {5, 10};

// std::vector<AutonStruct> AutonVector {
//   {5}
// };




int main() {
  vexcodeInit();
  Motor10.setMaxTorque(10, pct);
  Motor20.setMaxTorque(10, pct);
  double last_pos_motor10 = 0;
  double last_pos_motor20 = 0;
  double error_motor10 = 1;
  double loop_time = 5;
  Motor10.spin(fwd);
  Motor20.spin(fwd);

  while (1) {
    loop_time = test_auton_motor10.step_time_ms * error_motor10;
    Motor10.setVelocity((test_auton_motor10.step_pos_deg - last_pos_motor10) / loop_time, pct);
    Motor20.setVelocity((test_auton_motor20.step_pos_deg - last_pos_motor20) / loop_time, pct);

    // printf("Motor10 %f\n", test_auton_motor10.step_pos_deg);
    // printf("Motor20 %f\n", test_auton_motor20.step_pos_deg);
    error_motor10 = (test_auton_motor10.step_pos_deg - last_pos_motor10) / (Motor10.rotation(deg) - last_pos_motor10);
    // printf("Error 1 %f\n", test_auton_motor10.step_pos_deg - last_pos_motor10);
    // printf("Error 2 %f\n", Motor10.rotation(deg) - last_pos_motor10);

    last_pos_motor10 = test_auton_motor10.step_pos_deg;
    last_pos_motor20 = test_auton_motor20.step_pos_deg;


    test_auton_motor10.step_pos_deg += 10;
    test_auton_motor20.step_pos_deg += 10;

    printf("Time %f\n", loop_time);
    task::sleep(loop_time);
  }
}
