#include "main.h"

// #include <bits/stdc++.h>

enum BallColor {kRed, kBlue};

struct Ball {
  BallColor ball_color;
  double x;
  double y;
};

struct Goal {
  double x;
  double y;
  std::vector<BallColor> balls;
};

// std::vector<Ball> balls {
//   {5.8, 5.8, {{kBlue}, {kRed}}},
// };

std::vector<Ball> match_balls_on_field {
  {kBlue, 12,    12},
  {kBlue, 128.7, 12},
  {kRed,  70.3,  61.5},
  {kBlue, 35,    70.3},
  {kBlue, 61.5,  70.3},
  {kRed,  79.1,  70.3},
  {kRed,  105.7, 70.3},
  {kRed,  12,    12},
  {kRed,  128.7, 12},
  {kBlue, 70.3,  79.1},
};

std::vector<Ball> balls_on_field;

std::vector<Goal> match_goals {
  // {5.8, 5.8, {kBlue, kRed}},
  // {70.3, 5.8, {kBlue, kRed}},
  // {134.9, 5.8, {kBlue, kRed}},
  // {5.8, 70.3, {kBlue, kRed, kBlue}},
  // {134.9, 70.3, {kRed, kBlue, kRed}},
  // {5.8, 134.9, {kRed, kBlue}},
  {70.3, 134.9, {kRed, kBlue}},
  // {134.9, 134.9, {kRed, kBlue}},
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