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


  Goal::Goal(Point point, QAngle angle, GoalType goal_type) : point(point), angle(angle), offset(DEFAULT_GOAL_OFFSET), goal_type(goal_type) {
    goals.push_back(this);
  }

  Goal::Goal(Point point, QAngle angle, QLength offset, GoalType goal_type) : point(point), angle(angle), offset(offset), goal_type(goal_type) {
    goals.push_back(this);
  }

  Goal *Goal::closest(Point current_point) {
    Goal *closest_goal;
    for (auto &goal : goals) {
      if (OdomMath::computeDistanceToPoint(current_point, {goal->point.x, goal->point.y}) <
          OdomMath::computeDistanceToPoint(current_point, {closest_goal->point.x, closest_goal->point.y})) {
        closest_goal = goal;
      }
    }
    return closest_goal;
  }

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

Goal goal_1 ({  5.8129_in,   5.8129_in}, 0_deg, GoalType::kCorner);
Goal goal_2 ({  5.9272_in,  70.3361_in}, 0_deg, GoalType::kSide);
Goal goal_3 ({  5.8129_in, 134.8593_in}, 0_deg, GoalType::kCorner);
Goal goal_4 ({ 70.3361_in,   5.9272_in}, 0_deg, GoalType::kSide);
Goal goal_5 ({ 70.3361_in,  70.3361_in}, 0_deg, GoalType::kCenter);
Goal goal_6 ({ 70.3361_in,  134.745_in}, 0_deg, GoalType::kSide);
Goal goal_7 ({134.8593_in,   5.8129_in}, 0_deg, GoalType::kCorner);
Goal goal_8 ({ 134.745_in,  70.3361_in}, 0_deg, GoalType::kSide);
Goal goal_9 ({134.8593_in, 134.8593_in}, 0_deg, GoalType::kCorner);


namespace errorcorrection {

using namespace odomutilities;

Goal *last_goal;

bool first_goal_reached = false;

ObjectSensor goal_os ({&goal_sensor_one, &goal_sensor_two}, 2800, 2850);

bool waiting = false;
int time_triggered;
const int kWaitTime = 100;
const QLength kGoalOffset = 12.2274_in;
const QLength kDetectionDistance = 15_in;

void loop() {
  while (true) {
    bool goal_sensor_triggered = goal_os.get_new_found();
    bool goal_sensor_released = goal_os.get_new_lost();

    if (goal_sensor_triggered) {
      waiting = true;
      time_triggered = pros::millis();
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
        auto [distance_to_goal, angle_to_goal] = OdomMath::computeDistanceAndAngleToPoint(closest_goal_point, {measured_x, measured_y, odom.theta});

        if (distance_to_goal > kDetectionDistance + kGoalOffset) { // do nothing
        } else if (!first_goal_reached) {
          // controllermenu::partner_print_array[0] = "x " + std::to_string(closest_goal.x.convert(inch));
          // controllermenu::partner_print_array[1] = "y " + std::to_string(closest_goal.y.convert(inch));
          first_goal_reached = true;
          last_goal = closest_goal;
        } else if (closest_goal_point.x != last_point.x || closest_goal_point.y != last_point.y) {
          QAngle desired_angle = OdomMath::computeAngleToPoint(closest_goal_point, {last_point.x, last_point.y, 0_deg});
          Point measured_point = {measured_x, measured_y};
          QAngle measured_angle = OdomMath::computeAngleToPoint(measured_point, {last_point.x, last_point.y, 0_deg});
          // controllermenu::partner_print_array[0] = "d " + std::to_string(desired_angle.convert(degree)) + " x " + std::to_string(measured_x.convert(inch));
          // controllermenu::partner_print_array[1] = "m " + std::to_string(measured_angle.convert(degree)) + " y " + std::to_string(measured_y.convert(inch));

          QAngle error = measured_angle - desired_angle;
          QAngle new_theta = odom.theta - error;
          QLength new_x = closest_goal_point.x - kGoalOffset * cos(new_theta);
          QLength new_y = closest_goal_point.y - kGoalOffset * sin(new_theta);
          // controllermenu::partner_print_array[2] = "e " + std::to_string(error.convert(degree));
          if ((new_x - odom.x).abs() < 30_in
               && (new_y - odom.y).abs() < 30_in
               && (new_theta - odom.theta).abs() < 20_deg) {
            last_goal = closest_goal;
            chassis->setState({new_x, new_y, new_theta});
            autondrive::goal_center.start();
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