#ifndef UTILITIES
#define UTILITIES

// #include "main.h"
#define DEG_PER_SEC 21600
#define TRACKING_ORIGIN_OFFSET 1.5_in

#define DRIVER_SLEW 3
#define AUTON_SLEW 10


class Slew {
 public:
  Slew(double slew_rate) : slew_rate(slew_rate) {}

  double slew_rate;
  double new_value(double desired_value);
 private:
  double old_value = 0;
};

int pct_to_velocity(pros::Motor &motor);

template <typename T> int sgn(T &&val) {
  if (val < 0) {
    return -1;
  } else {
    return 1;
  }
};

struct RampMathSettings {
  int start_output;
  int mid_output;
  int end_output;
  double ramp_up_p;
  double ramp_down_p;
};

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


#endif // UTILITIES
