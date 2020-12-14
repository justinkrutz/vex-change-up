#include "json.hpp"
using json = nlohmann::ordered_json;

// extern json all_autons;
// extern std::string selected_auton_id;

std::string get_new_auton_id(json autons);
// void loadAllAutons();
// void jsonTest();
// void driveToClosestGoal();

class autonfromsd {
  public:
  autonfromsd(std::string auton_id);

  std::string auton_id;
  json auton_steps;
  int selected_step = 0;
  static json all_autons;

  void run();
  void save();
  void next_step();
  void previous_step();
  void set_step_drive_to_goal_and_score(int balls_in, int balls_out) ;
  void insert_step();
  void remove_step();
  void set_step_waypoint();
  static void load_autons_from_SD();
  static void save_autons_to_SD();
};