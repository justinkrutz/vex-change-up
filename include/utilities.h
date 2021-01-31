#ifndef UTILITIES
#define UTILITIES

// #include "main.h"
#define DEG_PER_SEC 21600
#define TRACKING_ORIGIN_OFFSET 1.5_in

#define DRIVER_SLEW 3
#define AUTON_SLEW 10

#define ARM_RANGE 170


class Slew {
 public:
  Slew(double slew_rate) : slew_rate(slew_rate) {}
  
  double new_value(double desired_value);
  double slew_rate;

 private:
  double old_value = 0;
};

class SlewGroup {

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

int rampMath(double input, double total_range, RampMathSettings s);

template <typename T>
bool InRange(T input, T min, T max) {
  return input >= MIN(min, max) && input <= MAX(min, max);
}

class ObjectSensor {
 public:
  
  ObjectSensor(std::vector<pros::ADILineSensor *> sensors, int found_threshold, int lost_threshold, bool starts_detected = false);

  int get_min_value();
  int get_max_value();
  bool get_new_found(bool additional_argument = true);
  bool get_new_lost(bool additional_argument = true);

  const std::vector<pros::ADILineSensor *> sensors;
  int found_threshold;
  int lost_threshold;
  bool is_detected;
  int time_when_found = 0;
  int time_when_lost = 0;
};

#endif // UTILITIES