#include "main.h"
#include "robot-config.h"
#include "robot-functions.h"
#include "auton-from-sd.h"
#include "auton-drive.h"
#include "controller-menu.h"

#include <bits/stdc++.h>
#include "json.hpp"
using json = nlohmann::ordered_json;

#define DEFAULT_GOAL_OFFSET 13_in
#define DEFAULT_BALL_OFFSET 10_in

std::string selected_auton_id = "0";


// double proximity (double x_one, double y_one, double x_two, double y_two) {
//   return fabs(sqrt(pow(x_one - x_two, 2) + pow(y_one - y_two, 2)));
// }

// template <typename T>
// T closestObject(QLength x, QLength y, std::vector<T> &objects) {
//   T closest_object;
//   for (auto &object : objects) {
//     if (proximity(x, y, object.x, object.y) <
//         proximity(x, y, closest_object.x, closest_object.y)) {
//       closest_object = object;
//     }
//   }
//   return closest_object;
// }

template <typename T>
T closestObject(QLength x, QLength y, std::vector<T> &objects) {
  T closest_object;
  for (auto &object : objects) {
    if (OdomMath::computeDistanceToPoint({x, y}, {object.x, object.y}) <
        OdomMath::computeDistanceToPoint({x, y}, {closest_object.x, closest_object.y})) {
      closest_object = object;
    }
  }
  return closest_object;
}



struct Ball {
  const QLength x;
  const QLength y;
  const QLength offset;
  enum Color {kOurColor, kOpposingColor};
  Color color;
  static std::vector<Ball*> Balls;

  Ball(QLength x, QLength y, Color color) : x(x), y(y), offset(DEFAULT_GOAL_OFFSET), color(color) {
    Balls.push_back(this);
  }
  Ball(QLength x, QLength y, QLength offset, Color color) : x(x), y(y), offset(offset), color(color) {
    Balls.push_back(this);
  }
};

struct Goal {
  const QLength x;
  const QLength y;
  const QLength offset;
  enum GoalType {kSide, kCorner, kCenter};
  GoalType goal_type;
  const std::vector<Ball::Color> balls;
  
  static std::vector<Goal*> goals;
  Goal(QLength x, QLength y, GoalType goal_type, std::vector<Ball::Color> balls) : x(x), y(y), offset(DEFAULT_GOAL_OFFSET), goal_type(goal_type), balls(balls) {
    goals.push_back(this);
  }
  Goal(QLength x, QLength y, QLength offset, GoalType goal_type, std::vector<Ball::Color> balls) : x(x), y(y), offset(offset), goal_type(goal_type), balls(balls) {
    goals.push_back(this);
  }
};

std::vector<Goal*> Goal::goals = {};

Goal closest_goal () {
  // QLength x = chassis->getState().x;
  // QLength y = chassis->getState().y;
  // return closestObject<Goal>(x, y, Goal::goals);
}

struct RobotPositionAtGoal {
  OdomState position;
  Goal goal;
};

std::vector<RobotPositionAtGoal> robot_positions_at_goals = {};

void update_odom(OdomState odom_state) {
  robot_positions_at_goals.back();
}



// std::vector<Ball> match_balls_on_field {
//   {12,    12,   Ball::kOurColor},
//   {128.7, 12,   Ball::kOurColor},
//   {70.3,  61.5, Ball::kOpposingColor},
//   {35,    70.3, Ball::kOurColor},
//   {61.5,  70.3, Ball::kOurColor},
//   {79.1,  70.3, Ball::kOpposingColor},
//   {105.7, 70.3, Ball::kOpposingColor},
//   {12,    12,   Ball::kOpposingColor},
//   {128.7, 12,   Ball::kOpposingColor},
//   {70.3,  79.1, Ball::kOurColor},
// };

std::vector<Ball> balls_on_field;

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

Goal goal_1 (5.8129_in,   5.8129_in,   Goal::GoalType::kCorner, {Ball::Color::kOurColor});
Goal goal_2 (5.9272_in,   70.3361_in,  Goal::GoalType::kSide,   {Ball::Color::kOurColor});
Goal goal_3 (5.8129_in,   134.8593_in, Goal::GoalType::kCorner, {Ball::Color::kOurColor});
Goal goal_4 (70.3361_in,  5.9272_in,   Goal::GoalType::kSide,   {Ball::Color::kOurColor});
Goal goal_5 (70.3361_in,  70.3361_in,  Goal::GoalType::kCenter, {Ball::Color::kOurColor});
Goal goal_6 (70.3361_in,  5.9272_in,   Goal::GoalType::kSide,   {Ball::Color::kOurColor});
Goal goal_7 (134.8593_in, 5.8129_in,   Goal::GoalType::kCorner, {Ball::Color::kOurColor});
Goal goal_8 (134.745_in,  70.3361_in,  Goal::GoalType::kSide,   {Ball::Color::kOurColor});
Goal goal_9 (134.8593_in, 134.8593_in, Goal::GoalType::kCorner, {Ball::Color::kOurColor});

std::vector<Point> goal_points {
  {5.8129_in,   5.8129_in},
  {5.9272_in,   70.3361_in},
  {5.8129_in,   134.8593_in},
  {70.3361_in,  5.9272_in},
  {70.3361_in,  70.3361_in},
  {70.3361_in,  134.745_in},
  {134.8593_in, 5.8129_in},
  {134.745_in,  70.3361_in},
  {134.8593_in, 134.8593_in}
};

// std::vector<Goal> goals;

void driveToClosestGoal(json step) {
  QLength x_pos = chassis->getState().x;
  QLength y_pos = chassis->getState().y;
  // Goal goal = closestObject<Goal>(x_pos.convert(inch), y_pos.convert(inch), match_goals);
  // chassis->driveToPoint({goal.x * inch, goal.y * inch}, false, 14_in);
  // chassis->driveToPoint({10 * inch, 10 * inch});
  // chassis->driveToPoint({70.3 * inch, 134.9 * inch}, false, 14_in);
}


// json all_autons;

// void loadAllAutons() {
//   std::ifstream i("/usd/autonomous_routines.json");
//   i >> all_autons;
//   i.close();
// }

std::string get_new_auton_id(json autons) {
  int largest_id_int = 0;
  for (auto & id_json : autons.items()) {
    int id_int = std::stoi(id_json.key(), nullptr);
    if (id_int > largest_id_int) {
      largest_id_int = id_int;
    }
  }
  return std::to_string(largest_id_int + 1);
}













json autonfromsd::all_autons = {};

void autonfromsd::load_autons_from_SD() {
  std::ifstream i("/usd/autonomous_routines.json");
  i >> all_autons;
  i.close();
}

void autonfromsd::save_autons_to_SD() {
  std::ofstream o("/usd/autonomous_routines.json");
  o << std::setw(2) << all_autons << std::endl;
  o.close();
}

autonfromsd::autonfromsd(std::string auton_id)
              : auton_id(auton_id), auton_steps(all_autons[auton_id]["steps"]){}

void drive_to_goal_and_cycle(std::string goal_id, int balls_in, int balls_out, QLength offset = 0_in) {
  // driveToClosestGoal
}

void autonfromsd::run() {
  json last_waypoint = {};
  for (auto& step : auton_steps.items()) {
    if (step.value()["stepType"] == "waypoint") {
      // double test = step.value()["x"];
      QLength x = step.value()["x"] * inch;
      QLength y = step.value()["y"] * inch;
      QAngle theta = step.value()["theta"] * degree;
      autondrive::drivetoposition::add_position_target(x, y, theta);
      // robotfunctions::driveToPosition(x, y, theta);
      last_waypoint = step.value();
    // }
    } else if (step.value()["stepType"] == "driveToBallAndIntake") {
      QLength x = last_waypoint["x"] * inch;
      QLength y = last_waypoint["y"] * inch;
      QAngle theta = last_waypoint["theta"] * degree;
      // robotfunctions::driveToPosition(x, y, theta);
      robotfunctions::intakeBalls(step.value()["ballsIn"]);
      robotfunctions::intakeBalls(step.value()["ballsOut"]);
    } else if (step.value()["stepType"] == "driveToGoalAndCycle") {
      // drive_to_goal_and_cycle(step.value()["goal"], step.value()["ballsIn"], step.value()["ballsOut"], );
    }
  }
}

void autonfromsd::save() {
  all_autons[auton_id]["name"] = "test auton";
  all_autons[auton_id]["autonType"] = "match";
  all_autons[auton_id]["steps"] = auton_steps;
}

void autonfromsd::next_step() {
  selected_step = std::min(selected_step + 1, int(auton_steps.size() - 1));
}

void autonfromsd::previous_step() {
  selected_step = std::max(selected_step - 1, 0);
}

void autonfromsd::set_step_drive_to_goal_and_score(int balls_in, int balls_out) {
  json step;
  step["stepType"] = "driveToGoalAndCycle";
  step["ballsIn"] = balls_in;
  step["ballsOut"] = balls_out;
  auton_steps[selected_step] = step;
}

void autonfromsd::insert_step() {
  json step = {};
  auton_steps.insert(auton_steps.begin() + ++selected_step, step);
}

void autonfromsd::remove_step() {
  auton_steps.erase(selected_step);
}

void autonfromsd::set_step_waypoint() {
  auton_steps[selected_step]["stepType"] = "waypoint";
  QLength x = chassis->getState().x;
  QLength y = chassis->getState().y;
  QAngle theta = chassis->getState().theta;
  auton_steps[selected_step]["x"] = x.convert(inch);
  auton_steps[selected_step]["y"] = y.convert(inch);
  auton_steps[selected_step]["theta"] = theta.convert(degree);
}

// void jsonTest() {
//   loadAllAutons();
//   std::cout << "all_autons before: " << all_autons.dump(2) << "\n";
//   json new_auton = all_autons["0"];
//   all_autons[get_new_auton_id(all_autons)] = new_auton;
//   std::ofstream o("/usd/autonomous_routines.json");
//   o << std::setw(2) << all_autons << std::endl;
//   o.close();
//   std::cout << "all_autons after: " << all_autons.dump(2) << "\n";
// }

namespace odomerrorcorrection {

Point last_point;

bool first_goal_reached = false;

bool goal_sensor_last = true;
bool waiting = false;
int time_triggered;
const int kWaitTime = 100;
const QLength kGoalOffset = 12.2274_in;
const QLength kDetectionDistance = 15_in;

void loop() {
  while (true) {
    bool goal_sensor_triggered = !goal_sensor_last && goal_sensor.get_value() < 2600;
    bool goal_sensor_released = goal_sensor.get_value() > 2800 && goal_sensor_last;

    if (goal_sensor_triggered) {
      goal_sensor_last = true;
      waiting = true;
      time_triggered = pros::millis();
    } else if (goal_sensor_released) {
      goal_sensor_last = false;
    }

    if (waiting && pros::millis() - time_triggered > kWaitTime) {
      waiting = false;
      if (goal_sensor.get_value() < 2600) {
        OdomState odom = chassis->getState();
        Point closest_goal = closestObject<Point>(odom.x, odom.y, goal_points);

        QLength measured_x = odom.x + cos(odom.theta) * kGoalOffset;
        QLength measured_y = odom.y + sin(odom.theta) * kGoalOffset;
        auto [distance_to_goal, angle_to_goal] = OdomMath::computeDistanceAndAngleToPoint(closest_goal, {measured_x, measured_y, odom.theta});

        if (distance_to_goal > kDetectionDistance + kGoalOffset) { // do nothing
        } else if (!first_goal_reached) {
          controllermenu::partner_print_array[0] = "x " + std::to_string(closest_goal.x.convert(inch));
          controllermenu::partner_print_array[1] = "y " + std::to_string(closest_goal.y.convert(inch));
          first_goal_reached = true;
          last_point = closest_goal;
        } else if (closest_goal.x != last_point.x || closest_goal.y != last_point.y) {
          QAngle desired_angle = OdomMath::computeAngleToPoint(closest_goal, {last_point.x, last_point.y, 0_deg});
          Point measured_point = {measured_x, measured_y};
          QAngle measured_angle = OdomMath::computeAngleToPoint(measured_point, {last_point.x, last_point.y, 0_deg});
          controllermenu::partner_print_array[0] = "d " + std::to_string(desired_angle.convert(degree)) + " x " + std::to_string(measured_x.convert(inch));
          controllermenu::partner_print_array[1] = "m " + std::to_string(measured_angle.convert(degree)) + " y " + std::to_string(measured_y.convert(inch));
          last_point = closest_goal;

          QAngle error = measured_angle - desired_angle;
          QAngle new_theta = odom.theta - error;
          QLength new_x = closest_goal.x - kGoalOffset * cos(new_theta);
          QLength new_y = closest_goal.y - kGoalOffset * sin(new_theta);
          controllermenu::partner_print_array[2] = "e " + std::to_string(error.convert(degree));
          chassis->setState({new_x, new_y, new_theta});
        }
      }
    }
    pros::delay(5);
  }

}

void start() {
  pros::Task task(loop);
}

}