#include "json.hpp"
using json = nlohmann::ordered_json;

extern json all_autons;

void loadAllAutons();
void driveToClosestGoal();
void jsonTest();