#ifndef ODOM_UTILITIES_H
#define ODOM_UTILITIES_H


namespace odomutilities {

enum GoalType {kSide, kCorner, kCenter};

class Goal {
 public:
  Goal(Point point, std::vector<QAngle> , GoalType goal_type);
  Goal(Point point, std::vector<QAngle> angles, QLength offset, GoalType goal_type);
  const Point point;
  const std::vector<QAngle> angles;
  const QLength offset;
  GoalType goal_type;

  static Goal *closest(Point current_point);
  static std::vector<Goal*> goals;
};

extern Goal goal_1;
extern Goal goal_2;
extern Goal goal_3;
extern Goal goal_4;
extern Goal goal_5;
extern Goal goal_6;
extern Goal goal_7;
extern Goal goal_8;
extern Goal goal_9;

namespace errorcorrection {

extern Point last_point;

void start();

}

} // namespace odomutilities

#endif // ODOM_UTILITIES_H
