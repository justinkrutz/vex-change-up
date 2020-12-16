#include "main.h"
#include "robot-functions.h"
#include "auton-from-sd.h"
#include "auton-controller.h"

#include <bits/stdc++.h>
#include "json.hpp"
using json = nlohmann::ordered_json;

std::string selected_auton_id = "0";

enum BallColor {kOurs, kTheirs };

struct Ball {
  double x;
  double y;
  BallColor ball_color;
};

struct Goal {
  double x;
  double y;
  std::vector<BallColor> balls;
};

std::vector<Ball> match_balls_on_field {
  {12,    12,   kOurs},
  {128.7, 12,   kOurs},
  {70.3,  61.5, kTheirs},
  {35,    70.3, kOurs},
  {61.5,  70.3, kOurs},
  {79.1,  70.3, kTheirs},
  {105.7, 70.3, kTheirs},
  {12,    12,   kTheirs},
  {128.7, 12,   kTheirs},
  {70.3,  79.1, kOurs},
};

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

#define DEFAULT_GOAL_OFFSET 12.13

json goal_coords = {
  {"1", {
    {"x", 5.8129},
    {"y", 5.8129},
    {"defaultAngle", 225},
    {"goalType", "corner"}
  }},
  {"2", {
    {"x", 5.9272},
    {"y", 70.3361},
    {"defaultAngle", 180},
    {"goalType", "side"}
  }},
  {"3", {
    {"x", 5.8129},
    {"y", 134.8593},
    {"defaultAngle", 135},
    {"goalType", "corner"}
  }},
  {"4", {
    {"x", 70.3361},
    {"y", 5.9272},
    {"defaultAngle", 315},
    {"goalType", "side"}
  }},
  {"5", {
    {"x", 70.3361},
    {"y", 70.3361},
    {"defaultAngle", 0},
    {"goalType", "center"}
  }},
  {"6", {
    {"x", 70.3361},
    {"y", 5.9272},
    {"defaultAngle", 315},
    {"goalType", "side"}
  }},
  {"7", {
    {"x", 134.8593},
    {"y", 5.8129},
    {"defaultAngle", 315},
    {"goalType", "corner"}
  }},
  {"8", {
    {"x", 134.745},
    {"y", 70.3361},
    {"defaultAngle", 0},
    {"goalType", "side"}
  }},
  {"9", {
    {"x", 134.8593},
    {"y", 134.8593},
    {"defaultAngle", 45},
    {"goalType", "corner"}
  }}
};

// std::vector<Goal> goals;

double proximity (double x_one, double y_one, double x_two, double y_two) {
  return fabs(sqrt(pow(x_one - x_two, 2) + pow(y_one - y_two, 2)));
}

template <typename T>
T closestObject(double x, double y, std::vector<T> objects) {
  T closest_object;
  for (auto &object : objects) {
    if (proximity(x, y, object.x, object.y) <
        proximity(x, y, closest_object.x, closest_object.y)) {
      closest_object = object;
    }
  }
  return closest_object;
}

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
      autoncontroller::drivetoposition::addPositionTarget(x, y, theta);
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
      drive_to_goal_and_cycle(step.value()["goal"], step.value()["ballsIn"], step.value()["ballsOut"], );
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