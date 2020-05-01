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
#include "data.h"


using namespace vex;

// #include <cstdio>
// #include <iostream>
// #include <iomanip>
// #include <fstream>
// #include <string>

#include <bits/stdc++.h>


AutonStruct test_auton_motor10 {5, 0.016666666666};
AutonStruct test_auton_motor20 {5, 0.016666666666};

void Test() {
  Motor10.setMaxTorque(20, pct);
  Motor20.setMaxTorque(20, pct);
  double last_pos_motor10 = 0;
  // double last_pos_motor20 = 0;
  // double error_motor10 = 1;
  double loop_time = 10;

  for (auto &this_step : auton_vector) {
    loop_time = this_step.step_time_ms;
    // Motor10.spin(fwd, this_step.step_speed_pct, pct);
    Motor10.spin(fwd, ((this_step.step_pos_deg / 2 - last_pos_motor10) / loop_time) * 166.6667, pct);
    // Motor10.startRotateTo(this_step.step_pos_deg, deg);
    // Motor20.spin(fwd, 100, pct);
    // Motor10.spin(fwd, 3 / loop_time, pct);
    // Motor20.spin(fwd, 3 / loop_time, pct);
    // // Motor10.setVelocity((test_auton_motor10.step_pos_deg - last_pos_motor10) / loop_time, pct);
    // // Motor20.setVelocity((test_auton_motor20.step_pos_deg - last_pos_motor20) / loop_time, pct);

    // // printf("Motor10 %f\n", test_auton_motor10.step_pos_deg);
    // // printf("Motor20 %f\n", test_auton_motor20.step_pos_deg);
    // error_motor10 = (test_auton_motor10.step_pos_deg - last_pos_motor10) / (Motor10.rotation(deg) - last_pos_motor10);
    // printf("Error 1 %f\n", test_auton_motor10.step_pos_deg);
    // printf("Error 2 %f\n", last_pos_motor10);
    // printf("Error 3 %f\n", Motor10.rotation(deg));

    last_pos_motor10 = this_step.step_pos_deg / 2;
    // last_pos_motor20 = test_auton_motor20.step_pos_deg;


    // test_auton_motor10.step_pos_deg += 0.016666666666;
    // test_auton_motor20.step_pos_deg += 0.016666666666;

    printf("Error %f\n", Motor10.rotation(deg) - this_step.step_pos_deg / 2);
    task::sleep(loop_time);
  }
  Motor10.stop();
}

void CreateData() {
  double last_time = Brain.Timer.time(msec) - 5;
  repeat(1000) {
    printf("{%f, %f, %f},\r\n", Brain.Timer.time(msec) - last_time, Motor10.rotation(deg), Motor10.velocity(pct));
    last_time = Brain.Timer.time(msec);
    task::sleep(20);
  }
}

int main() {
  vexcodeInit();
  Test();
  // thread thread1(CreateData);
  while (1) {
    // Motor10.spin(fwd, Controller1.Axis3.position(), pct);
    task::sleep(10);
  }
}
