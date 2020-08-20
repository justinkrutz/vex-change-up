#include "main.h"
#include "robot-functions.h"
#include "autonomous.h"

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

/*     opposing alliance side
   │             │
   └─────────────┘
┌───────────────────┐
│7        8        9│
│─────────╩─────────│
│                   │
│4════════5════════6│
│                   │
│─────────╦─────────│
│1        2        3│
└───────────────────┘
   ┌─────────────┐
   │             │
      alliance     */


std::vector<Goal> match_goals {
  {5.8, 5.8, {kOurs, kTheirs}},
  {70.3, 5.8, {kOurs, kTheirs}},
  {134.9, 5.8, {kOurs, kTheirs}},
  {5.8, 70.3, {kOurs, kTheirs, kOurs}},
  {70.3, 70.3, {}},
  {134.9, 70.3, {kTheirs, kOurs, kTheirs}},
  {5.8, 134.9, {kTheirs, kOurs}},
  {70.3, 134.9, {kTheirs, kOurs}},
  {134.9, 134.9, {kTheirs, kOurs}},
};

std::vector<Goal> skills_goals {
  {5.8, 5.8, {kTheirs, kTheirs}},
  {70.3, 5.8, {kTheirs}},
  {134.9, 5.8, {kTheirs, kTheirs}},
  {5.8, 70.3, {kTheirs}},
  {70.3, 70.3, {kTheirs, kTheirs, kTheirs}},
  {134.9, 70.3, {kTheirs}},
  {5.8, 134.9, {kTheirs, kTheirs}},
  {70.3, 134.9, {kTheirs}},
  {134.9, 134.9, {kTheirs, kTheirs}},
};

std::vector<Goal> goals;

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

void driveToClosestGoal() {
  QLength x_pos = chassis->getState().x;
  QLength y_pos = chassis->getState().y;
  Goal goal = closestObject<Goal>(x_pos.convert(inch), y_pos.convert(inch), match_goals);
  chassis->driveToPoint({goal.x * inch, goal.y * inch}, false, 14_in);
  // chassis->driveToPoint({10 * inch, 10 * inch});
  // chassis->driveToPoint({70.3 * inch, 134.9 * inch}, false, 14_in);
}


// json all_autons;

// void loadAllAutons() {
//   std::ifstream i("/usd/autonomous_routines.json");
//   i >> all_autons;
//   i.close();
// }

std::string getNewAutonId(json autons) {
  int largest_id_int = 0;
  for (auto & id_json : autons.items()) {
    int id_int = std::stoi(id_json.key(), nullptr);
    if (id_int > largest_id_int) {
      largest_id_int = id_int;
    }
  }
  return std::to_string(largest_id_int + 1);
}













json AutonManager::all_autons = {};

void AutonManager::loadAutonsFromSD() {
  std::ifstream i("/usd/autonomous_routines.json");
  i >> all_autons;
  i.close();
}

void AutonManager::saveAutonsToSD() {
  std::ofstream o("/usd/autonomous_routines.json");
  o << std::setw(2) << all_autons << std::endl;
  o.close();
}

AutonManager::AutonManager(std::string auton_id)
              : auton_id(auton_id), auton_steps(all_autons[auton_id]["steps"]){}

void AutonManager::run() {
  json last_waypoint = {};
  for (auto& step : auton_steps.items()) {
    if (step.value()["stepType"] == "waypoint") {
      // double test = step.value()["x"];
      QLength x = step.value()["x"] * inch;
      QLength y = step.value()["y"] * inch;
      QAngle theta = step.value()["theta"] * degree;
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
      driveToClosestGoal();
    }
  }
}

void AutonManager::save() {
  all_autons[auton_id]["name"] = "test auton";
  all_autons[auton_id]["autonType"] = "match";
  all_autons[auton_id]["steps"] = auton_steps;
}

void AutonManager::nextStep() {
  selected_step = std::min(selected_step + 1, int(auton_steps.size() - 1));
}

void AutonManager::previousStep() {
  selected_step = std::max(selected_step - 1, 0);
}

void AutonManager::setStepDriveToGoalAndScore(int balls_in, int balls_out) {
  json step;
  step["stepType"] = "driveToGoalAndCycle";
  step["ballsIn"] = balls_in;
  step["ballsOut"] = balls_out;
  auton_steps[selected_step] = step;
}

void AutonManager::insertStep() {
  json step = {};
  auton_steps.insert(auton_steps.begin() + ++selected_step, step);
}

void AutonManager::removeStep() {
  auton_steps.erase(selected_step);
}

void AutonManager::setStepWaypoint() {
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
//   all_autons[getNewAutonId(all_autons)] = new_auton;
//   std::ofstream o("/usd/autonomous_routines.json");
//   o << std::setw(2) << all_autons << std::endl;
//   o.close();
//   std::cout << "all_autons after: " << all_autons.dump(2) << "\n";
// }