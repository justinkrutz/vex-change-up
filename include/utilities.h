#ifndef UTILITIES
#define UTILITIES

// #include "main.h"
#define DEG_PER_SEC 21600
#define TRACKING_ORIGIN_OFFSET 1.5_in

#define DRIVER_SLEW 3
#define AUTON_SLEW 10

template <typename T> int sgn(T &&val) {
  if (val < 0) {
    return -1;
  } else {
    return 1;
  }
};

class Slew {
 public:
  Slew(double slew_rate) : slew_rate(slew_rate) {}

  double slew_rate;
  double new_value(double desired_value);
 private:
  double old_value = 0;
};

int pct_to_velocity(pros::Motor &motor);


#endif // UTILITIES
