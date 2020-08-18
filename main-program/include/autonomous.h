#include "json.hpp"
using json = nlohmann::ordered_json;

// extern json all_autons;
// extern std::string selected_auton_id;

std::string getNewAutonId(json autons);
// void loadAllAutons();
// void jsonTest();
// void driveToClosestGoal();

class AutonManager {
  public:
  AutonManager(std::string auton_id);

  std::string auton_id;
  json auton_steps;
  int selected_step = 0;
  static json all_autons;

  void run();
  void save();
  void nextStep();
  void previousStep();
  void setStepDriveToGoalAndScore(int balls_in, int balls_out) ;
  void insertStep();
  void removeStep();
  void setStepWaypoint();
  static void loadAutonsFromSD();
  static void saveAutonsToSD();
};