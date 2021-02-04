#include "main.h"
#include "robot-config.h"
#include "robot-functions.h"
#include "odom-utilities.h"
#include "auton-drive.h"
#include "controller-menu.h"

#include <bits/stdc++.h>
#include "json.hpp"
using json = nlohmann::ordered_json;

#define DEFAULT_GOAL_OFFSET 13_in
#define DEFAULT_BALL_OFFSET 10_in

namespace odomutilities {


Goal::Goal(Point point, std::vector<QAngle> angles, GoalType goal_type) : point(point), angles(angles), offset(DEFAULT_GOAL_OFFSET), goal_type(goal_type) {
  goals.push_back(this);
}

Goal::Goal(Point point, std::vector<QAngle> angles, QLength offset, GoalType goal_type) : point(point), angles(angles), offset(offset), goal_type(goal_type) {
  goals.push_back(this);
}

std::vector<Goal*> Goal::goals = {};

Goal *Goal::closest(Point current_point) {
  Goal *closest_goal = goals[0];
  for (auto &&goal : goals) {
    if (OdomMath::computeDistanceToPoint(current_point, {goal->point.x, goal->point.y}) <
        OdomMath::computeDistanceToPoint(current_point, {closest_goal->point.x, closest_goal->point.y})) {
      closest_goal = goal;
    }
  }
  return closest_goal;
}

Goal goal_1 ({  5.8129_in,   5.8129_in}, {225_deg}, GoalType::kCorner);
Goal goal_2 ({  5.9272_in,  70.3361_in}, {180_deg}, GoalType::kSide);
Goal goal_3 ({  5.8129_in, 134.8593_in}, {135_deg}, GoalType::kCorner);
Goal goal_4 ({ 70.3361_in,   5.9272_in}, {270_deg}, GoalType::kSide);
Goal goal_5 ({ 70.3361_in,  70.3361_in}, {  0_deg, 90_deg, 180_deg, 270_deg}, GoalType::kCenter);
Goal goal_6 ({ 70.3361_in, 134.7450_in}, { 90_deg}, GoalType::kSide);
Goal goal_7 ({134.8593_in,   5.8129_in}, {315_deg}, GoalType::kCorner);
Goal goal_8 ({134.7450_in,  70.3361_in}, {  0_deg}, GoalType::kSide);
Goal goal_9 ({134.8593_in, 134.8593_in}, { 45_deg}, GoalType::kCorner);


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
  Point ball_a { 12.0304_in,  12.0304_in};
  Point ball_b { 12.0304_in, 128.6418_in};
  Point ball_c { 61.5432_in,  70.3361_in};
  Point ball_d { 70.3361_in,  34.9916_in};
  Point ball_e { 70.3361_in,  61.5432_in};
  Point ball_f { 70.3361_in,  79.1290_in};
  Point ball_g { 70.3361_in, 105.6806_in};
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
  Point ball_a { 22.8361_in,  34.9911_in};
  Point ball_b { 22.8361_in, 105.8361_in};
  Point ball_c { 34.9911_in,   3.3361_in};
  Point ball_d { 34.9911_in, 137.3361_in};
  Point ball_e { 46.8361_in,  70.3361_in};
  Point ball_f { 70.3361_in,  23.3361_in};
  Point ball_g { 70.3361_in,  46.8361_in};
  Point ball_h { 70.3361_in,  93.8361_in};
  Point ball_i { 70.3361_in, 117.3361_in};
  Point ball_j { 93.8361_in,  70.3361_in};
  Point ball_k {105.8361_in,   3.3361_in};
  Point ball_l {105.8361_in, 137.3361_in};
  Point ball_m {117.6361_in,  34.9911_in};
  Point ball_n {117.6361_in, 105.6361_in};
}

namespace errorcorrection {

using namespace odomutilities;

Goal *last_goal = &goal_9;

bool first_goal_reached = false;

ObjectSensor goal_os ({&goal_sensor_one, &goal_sensor_two}, 2800, 2850);

bool waiting = false;
int time_triggered = 0;
const int kWaitTime = 100;
const QLength kGoalOffset = 12.2274_in;
const QLength kDetectionDistance = 15_in;

bool auto_goal_center;

void loop() {
  while (true) {
    bool goal_sensor_triggered = goal_os.get_new_found();
    bool goal_sensor_released = goal_os.get_new_lost();

    if (goal_sensor_triggered) {
      waiting = true;
      time_triggered = pros::millis();
      // if (auto_goal_center) autondrive::goal_center.start();
    }

    if (waiting && pros::millis() - time_triggered > kWaitTime) {
      waiting = false;
      if (goal_os.is_detected) {
        OdomState odom = chassis->getState();
        Goal *closest_goal = Goal::closest({odom.x, odom.y});
        Point last_point = last_goal->point;
        Point closest_goal_point = closest_goal->point;

        QLength measured_x = odom.x + cos(odom.theta) * kGoalOffset;
        QLength measured_y = odom.y + sin(odom.theta) * kGoalOffset;
        QLength distance_to_goal = OdomMath::computeDistanceToPoint(closest_goal_point, {measured_x, measured_y, odom.theta});

        if (distance_to_goal > kDetectionDistance) {
          continue;
        }

        if (!first_goal_reached) {
          first_goal_reached = true;
          last_goal = closest_goal;
        } else if (closest_goal_point.x != last_point.x || closest_goal_point.y != last_point.y) {
          QAngle desired_angle = OdomMath::computeAngleToPoint(closest_goal_point, {last_point.x, last_point.y, 0_deg});
          Point measured_point = {measured_x, measured_y};
          QAngle measured_angle = OdomMath::computeAngleToPoint(measured_point, {last_point.x, last_point.y, 0_deg});
          controllermenu::partner_print_array[0] = "d " + std::to_string(desired_angle.convert(degree)) + " x " + std::to_string(measured_x.convert(inch));
          controllermenu::partner_print_array[1] = "m " + std::to_string(measured_angle.convert(degree)) + " y " + std::to_string(measured_y.convert(inch));

          QAngle error = mod(measured_angle - desired_angle + 180_deg, 360_deg) - 180_deg;

          QAngle new_theta = odom.theta - error;
          QLength new_x = closest_goal_point.x - kGoalOffset * cos(new_theta);
          QLength new_y = closest_goal_point.y - kGoalOffset * sin(new_theta);
          controllermenu::partner_print_array[2] = "e " + std::to_string(error.convert(degree));
          if ((new_x - odom.x).abs() < 30_in
               && (new_y - odom.y).abs() < 30_in
               && (new_theta - odom.theta).abs() < 20_deg) {
            last_goal = closest_goal;
            chassis->setState({new_x, new_y, new_theta});
          }
        }
      }
    }
    pros::delay(5);
  }

}

void start() {
  pros::Task task(loop);
}

} // namespace errorcorrection

} // namespace odomutilities