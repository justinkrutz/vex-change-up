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

/*        MATCH SETUP
   │                       │
   └───────────────────────┘
┌─────────────────────────────┐
│                             │
│──────────────╩──────────────│
│                             │
│                             │
│                             │
│4═════D═════E═5═F═════G═════6│
│              C              │
│                             │
│                             │
│──A───────────╦───────────B──│
│1             2             3│
└─────────────────────────────┘
   ┌───────────────────────┐
   │                       │
*/

namespace matchballs {
  extern Point ball_a;
  extern Point ball_b;
  extern Point ball_c;
  extern Point ball_d;
  extern Point ball_e;
  extern Point ball_f;
  extern Point ball_g;
}

/*       SKILLS SETUP
   │                       │
   └───────────────────────┘
┌──────────────╦──────────────┐
│7             8             9│
│──────M───────╩───────N──────│
│K                           L│
│              J              │
│                             │
│4═══F════G════5════H════I═══6│
│                             │
│              E              │
│C                           D│
│──────A───────╦───────B──────│
│1             2             3│
└──────────────╩──────────────┘
   ┌───────────────────────┐
   │                       │
*/

namespace skillsballs {
  extern Point ball_a;
  extern Point ball_b;
  extern Point ball_c;
  extern Point ball_d;
  extern Point ball_e;
  extern Point ball_f;
  extern Point ball_g;
  extern Point ball_h;
  extern Point ball_i;
  extern Point ball_j;
  extern Point ball_k;
  extern Point ball_l;
  extern Point ball_m;
  extern Point ball_n;
}

namespace errorcorrection {

extern Point last_point;

extern bool auto_goal_center;

void start();

}

} // namespace odomutilities

#endif // ODOM_UTILITIES_H
