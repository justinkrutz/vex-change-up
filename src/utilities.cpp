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

int rampMath(double input, double total_range, RampMathSettings s) {
  input = fabs(input);
  int sign = sgn(total_range);
  total_range = fabs(total_range);
  int start_output = abs(s.start_output);
  int mid_output = abs(s.mid_output);
  int end_output = abs(s.end_output);
  double ramp_up_p = fabs(s.ramp_up_p);
  double ramp_down_p = fabs(s.ramp_down_p);

  double ramp_up_range = (mid_output - start_output)*ramp_up_p;
  double ramp_down_range = (mid_output - end_output)*ramp_down_p;
  double ramp_range_multiplier = std::min(1.0, total_range / (ramp_up_range + ramp_down_range));
  if (start_output != mid_output && input < ramp_up_range * ramp_range_multiplier) {
    return (std::max(0, (int)round(input / ramp_up_p)) + start_output) * sign;
  } else if (end_output != mid_output && input > (total_range - ramp_down_range) * ramp_range_multiplier) {
    return (std::max(0, (int)round((total_range - input) / ramp_down_p)) + end_output) * sign;
  }
  return mid_output * sign;
}