#include "main.h"
#include "utilities.h"

double Slew::new_value(double desired_value) {
  double new_value;
  if (desired_value > old_value + slew_rate)
    new_value = old_value + slew_rate;
  else if (desired_value < old_value - slew_rate)
    new_value = old_value - slew_rate;
  else {
    new_value = desired_value;
  }
  return old_value = new_value;
}

int pct_to_velocity(pros::Motor &motor) {
  switch (motor.get_gearing()) {
    case pros::E_MOTOR_GEARSET_36:
      return 1;
    case pros::E_MOTOR_GEARSET_06:
      return 6;
    case pros::E_MOTOR_GEARSET_18:
    default:
      return 2;
  }
}
