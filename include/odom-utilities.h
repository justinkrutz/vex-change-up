#ifndef ODOM_UTILITIES_H
#define ODOM_UTILITIES_H


namespace odomutilities {

enum GoalType {kSide, kCorner, kCenter};

class Goal {
 public:
  Goal(Point point, QAngle angle, GoalType goal_type);
  Goal(Point point, QAngle angle, QLength offset, GoalType goal_type);
  const Point point;
  const QAngle angle;
  const QLength offset;
  GoalType goal_type;

  static Goal *closest(Point current_point);
  static std::vector<Goal*> goals;
};

namespace errorcorrection {

extern Point last_point;

void start();

}

} // namespace odomutilities

#endif // ODOM_UTILITIES_H
