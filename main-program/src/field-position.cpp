#include "api.h"

#include "robot-config.h"

enum BallColor {kRed, kBlue};

struct Ball {
  BallColor ball_color;
  double x;
  double y;
};

struct Goal {
  double x;
  double y;
  std::vector<Ball> balls;
};

// std::vector<Ball> balls {
//   {5.8, 5.8, {{kBlue}, {kRed}}},
// };

std::vector<Ball> balls_on_field {
  {kBlue, 5.8, 5.8},
  {kRed,  5.8, 5.8},

  {kBlue, 70.3, 5.8},
  {kRed,  70.3, 5.8},

  {kBlue, 134.9, 5.8},
  {kRed,  134.9, 5.8},

  {kBlue, 12, 12},

  {kBlue, 128.7, 12},

  {kRed,  70.3,  61.5},

  {kBlue, 5.8, 70.3},
  {kRed,  5.8, 70.3},
  {kBlue, 5.8, 70.3},

  {kBlue, 35, 70.3},
  
  {kBlue, 61.5, 70.3},

  {kRed,  79.1, 70.3},

  {kRed,  105.7, 70.3},

  {kRed,  134.9, 70.3},
  {kBlue, 134.9, 70.3},
  {kRed,  134.9, 70.3},
  

  {kRed,  5.8, 134.9},
  {kBlue, 5.8, 134.9},

  {kRed,  70.3, 134.9},
  {kBlue, 70.3, 134.9},

  {kRed,  134.9, 134.9},
  {kBlue, 134.9, 134.9},

  {kRed,  12, 12},

  {kRed,  128.7, 12},

  {kBlue, 70.3, 79.1},
};

Ball test_ball;

std::vector<Goal> goals {
  Goal{5.8, 5.8, {test_ball}}
  // {{5.8, 5.8, kBlue},
  // {5.8, 5.8, kRed}},
  
  // {{70.3, 5.8, kBlue},
  // {70.3, 5.8, kRed}},
  
  // {{134.9, 5.8, kBlue},
  // {134.9, 5.8, kRed}},
  
  // {{5.8, 70.3, kBlue},
  // {5.8, 70.3, kRed},
  // {5.8, 70.3, kBlue}},
  
  // {{134.9, 70.3, kRed},
  // {134.9, 70.3, kBlue},
  // {134.9, 70.3, kRed}}
  

  // {{5.8, 134.9, kRed},
  // {5.8, 134.9, kBlue}},
  
  // {{70.3, 134.9, kRed},
  // {70.3, 134.9, kBlue}},
  
  // {{134.9, 134.9, kRed},
  // {134.9, 134.9, kBlue}}
};