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


class PID {
  public:
    PID(double set_kp, double set_ki, double set_kd) {
      kp = set_kp;
      ki = set_ki;
      kd = set_kd;
    }

    double out(double error) {
      double proportional = kp * error;
      integral = ((((error + last_error) / 2) * last_time) + integral) * ki;
      double derivative = ((error - last_error) / last_time) * kd;

      last_time.reset();
      last_error = error;
      return proportional + integral + derivative;
    }
  private:
    double kp;
    double ki;
    double kd;
    double last_error;
    double integral;
    vex::timer last_time;
};

PID autonPID(1, 0, 0);

void RunAuton() {
  // Motor10.setMaxTorque(20, pct);
  // Motor20.setMaxTorque(20, pct);
  double last_pos_motor10 = 0;
  // double last_pos_motor20 = 0;
  double error_motor10 = 1;
  double loop_time;
  // double time_multiplier;

  for (auto &this_step : auton_vector) {
    double temp_error = fabs(last_pos_motor10) / fabs(Motor10.rotation(deg));
    if (std::isnormal(temp_error)) {
    // error_motor10 = (temp_error);
    }
    error_motor10 = (1);
    loop_time = this_step.step_time_ms * error_motor10;
    
    Motor10.spin(fwd, (autonPID.out((this_step.step_pos_deg - Motor10.rotation(deg)))) + (this_step.step_pos_deg - last_pos_motor10) * (166.6667 / loop_time), pct);
    // Motor10.spin(fwd, ((last_pos_motor10 - Motor10.rotation(deg)) * 1) + this_step.step_speed_pct, pct);
    // Motor10.spin(fwd, /* ((last_pos_motor10 - Motor10.rotation(deg)) * 1) +  */(this_step.step_pos_deg - last_pos_motor10) * (166.6667 / loop_time), pct);


    last_pos_motor10 = this_step.step_pos_deg;

    printf("Error %f\r\n", this_step.step_pos_deg - Motor10.rotation(deg));
    // printf("%f\t%f\t%f\t%f\t%f\r\n", Motor10.rotation(deg), last_pos_motor10, this_step.step_pos_deg, Motor10.power(), this_step.step_speed_pct);
    task::sleep(loop_time);
  }
  Motor10.stop();
}

double speed = 0.2;

void CreateData() {
  int loop_time = 100;
  repeat(1000) {
    printf("  {%f, %f, %f},\r\n", loop_time * speed, Motor10.rotation(deg), Motor10.velocity(pct));
    task::sleep(loop_time);
  }
}

void ManualControl() {
  vex::timer LoopTime;
  double stickForwardTemp = 0;
  double stickForwardLast = 0;
  double slew_multiplier = 0.1 * speed;
  while (1) {
    stickForwardTemp = Controller1.Axis3.position() * speed;
    if(stickForwardTemp > stickForwardLast + LoopTime * slew_multiplier) {
      stickForwardTemp = stickForwardLast + LoopTime * slew_multiplier;
    } else if(stickForwardTemp < stickForwardLast - LoopTime * slew_multiplier) {
      stickForwardTemp = stickForwardLast - LoopTime * slew_multiplier;
    }

    stickForwardLast = stickForwardTemp;
    LoopTime.reset();

    Motor10.spin(fwd, stickForwardTemp, pct);
    task::sleep(10);
  }
}

int main() {
  vexcodeInit();
  // RunAuton();
  thread thread1(CreateData);
  thread thread2(ManualControl);
  while (1) {
    task::sleep(100);
  }
}
