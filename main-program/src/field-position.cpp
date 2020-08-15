#include "main.h"

// #include <bits/stdc++.h>
#include "json.hpp"

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