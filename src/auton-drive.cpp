#include "main.h"

#include "utilities.h"
#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"
#include "auton-drive.h"
#include "odom-utilities.h"
#include <stdio.h>
#include <complex.h>
#include "odometry.h"


namespace autondrive {

controllerbuttons::MacroGroup auton_group;
controllerbuttons::MacroGroup drive_group;

OdomState get_odom_state() {
  return imu_odom->getState();
}

double button_strafe = 0;
double button_turn = 0;
double button_forward = 0;

namespace drivetoposition {

Target::Target(QLength x, QLength y, QAngle theta) : x(x), y(y), theta(theta) {}

void Target::init_if_new() {
  if (is_new) {
    is_new = false;
    starting_state = get_odom_state();
  }
}

std::queue<Target> targets;
std::queue<Target> target_queue;

bool auton_drive_enabled = true;
bool targets_should_clear = true;
bool final_target_reached = true;
bool target_heading_reached = false;
bool target_distance_reached = false;

OdomState starting_position;
// RampMathSettings move_settings = {20, 100, 20, 0.5, 0.5};
// RampMathSettings turn_settings = {20, 100, 20, 0.1, 0.1};
RampMathSettings move_settings = {20, 100, 15, 0.1, 0.2};
RampMathSettings turn_settings = {10, 50, 10, 0.1, 0.1};

double forward = 0;
double strafe  = 0;
double turn    = 0;


void update() {
  if (targets_should_clear) {
    targets = {};
  }

  while (!target_queue.empty()) { 
    targets.push(target_queue.front());
    target_queue.pop();
    targets_should_clear = false;
  }

  if (targets.empty() || !auton_drive_enabled) {
    forward = 0;
    strafe = 0;
    turn = 0;
    final_target_reached = true;
    return;
  }

  Target &target = targets.front();
  target.init_if_new();

  double move_speed;
  double turn_speed;

  Point target_point{target.x, target.y};

  OdomState target_state{target.x, target.y, target.theta};
  Point starting_point{target.starting_state.x, target.starting_state.y};

  auto [distance_to_target, direction] = OdomMath::computeDistanceAndAngleToPoint(target_point, get_odom_state());
  QLength distance_traveled = OdomMath::computeDistanceToPoint(starting_point, get_odom_state());
  QLength total_distance = OdomMath::computeDistanceToPoint(starting_point, target_state);

  QAngle total_angle = target.theta - target.starting_state.theta;


  if (distance_traveled >= total_distance) {
    target_distance_reached = true;
  }

  if (abs(get_odom_state().theta - target_state.theta) < 2.5_deg) {
    target_heading_reached = true;
  }

  if (target_heading_reached && target_distance_reached) {
    if (!targets.empty()) {
      target_heading_reached = false;
      target_distance_reached = false;
      targets.pop();
      return;
    } else {
      final_target_reached = true;
    }
  } 
  
  if (target_distance_reached) {
    move_speed = std::min(100.0, 5 * distance_to_target.convert(inch));
  } else {
    move_speed = rampMath(distance_traveled.convert(inch), total_distance.convert(inch), move_settings);
  }

  if (target_heading_reached) {
    turn_speed = std::min(100.0, 100 * (target.theta - get_odom_state().theta).convert(radian));
  } else {
    turn_speed = std::min(100.0, 100 * (target.theta - get_odom_state().theta).convert(radian));
  }

  forward = move_speed * cos(direction.convert(radian));
  strafe  = move_speed * sin(direction.convert(radian));
  turn    = turn_speed;
}

void add_target(QLength x, QLength y, QAngle theta, QLength offset_distance, QAngle offset_angle) {
  QLength x_offset = cos(offset_angle) * offset_distance;
  QLength y_offset = sin(offset_angle) * offset_distance;
  final_target_reached = false;
  target_queue.push({x - x_offset, y - y_offset, theta});
}

void add_target(QLength x, QLength y, QAngle theta, QLength offset_distance) {
  add_target(x, y, theta, offset_distance, theta);
}

void add_target(QLength x, QLength y, QAngle theta) {
  add_target(x, y, theta, 0_in);
}

void add_target(odomutilities::Goal goal, QAngle theta, QLength offset_distance, QAngle offset_angle) {
  add_target(goal.point.x, goal.point.y, theta, offset_distance, offset_angle);
}

void add_target(odomutilities::Goal goal, QAngle theta, QLength offset_distance) {
  add_target(goal, theta, offset_distance, theta);
}

void add_target(odomutilities::Goal goal, QAngle theta) {
  add_target(goal, theta, 0_in);
}

void add_target(Point point, QAngle theta, QLength offset_distance, QAngle offset_angle) {
  add_target(point.x, point.y, theta, offset_distance, offset_angle);
}

void add_target(Point point, QAngle theta, QLength offset_distance) {
  add_target(point, theta, offset_distance, theta);
}

void add_target(Point point, QAngle theta) {
  add_target(point, theta, 0_in);
}

void clear_all_targets() {
  targets_should_clear = true;
}

void eject_all_but(int balls_to_keep) {
  using namespace robotfunctions::rollers;
  eject_queue = balls_in_robot.size() - balls_to_keep;
}

void splay_intakes_if_running() {
  using namespace robotfunctions;
  if (rollers::intake_queue > 0) {
    intake_splay();
  }
}

using namespace controllerbuttons;

// blocking functions

void wait_until_final_target_reached() {
  while (!final_target_reached) {
    wait(10);
  }
}

QAngle goal_center_angle;

Macro goal_center(
    [&](){
      int time = pros::millis();

      while (abs(get_odom_state().theta - goal_center_angle) > 1_deg && pros::millis() - time < 2000) {
        double speed = 2 * (goal_center_angle - get_odom_state().theta).convert(degree);
        button_strafe = -speed;
        button_turn = speed * 0.7;
        button_forward = 0.04 * (MIN(goal_sensor_one.get_value(), goal_sensor_two.get_value() - 2400));
        wait(10);
      }
    },
    [](){
        button_strafe = 0;
        button_turn = 0;
        button_forward = 0;
    },
    {&drive_group, &auton_group});

void drive_to_goal(odomutilities::Goal goal, QAngle angle, int timeout = 2000) {
  ObjectSensor goal_os ({&goal_sensor_one, &goal_sensor_two}, 2600, 2750);
  add_target(goal.point.x, goal.point.y, angle, 12.4_in);
  int time = pros::millis();
  while (!goal_os.is_detected && pros::millis() - time < timeout) {
    goal_os.get_new_found();
    goal_os.get_new_lost();
    if (final_target_reached) {
      clear_all_targets();
      button_forward = 20;
    }
    wait(10);
  }
  clear_all_targets();
  // button_forward = 0;
  goal_center_angle = angle;
  goal_center.start();
}

void score_balls(int balls_to_score) {
  using namespace robotfunctions::rollers;
  for (int i = 0; i < balls_to_score; i++) {
    score_queue++;
    wait(200);
  }
  while (!score_queue == 0) {
    wait(10);
  }
}

} // namespace drivetoposition


void motor_task()
{
  std::shared_ptr<AbstractMotor> drive_fl = x_model->getTopLeftMotor();
  std::shared_ptr<AbstractMotor> drive_fr = x_model->getTopRightMotor();
  std::shared_ptr<AbstractMotor> drive_bl = x_model->getBottomLeftMotor();
  std::shared_ptr<AbstractMotor> drive_br = x_model->getBottomRightMotor();

  // int time_last = 0;
  Slew drive_fl_slew(DRIVER_SLEW);
  Slew drive_fr_slew(DRIVER_SLEW);
  Slew drive_bl_slew(DRIVER_SLEW);
  Slew drive_br_slew(DRIVER_SLEW);

  while(1)
  {
    drivetoposition::update();

    double forward = button_forward + drivetoposition::forward + master.get_analog(ANALOG_RIGHT_Y) * 0.787401574803;
    // double strafe  = button_strafe + drivetoposition::strafe;
    double strafe  = button_strafe + drivetoposition::strafe  + master.get_analog(ANALOG_LEFT_X) * 0.787401574803;
    // double strafe  = button_strafe + drivetoposition::strafe  + master.get_analog(ANALOG_RIGHT_X) * 0.787401574803;
    double temp_turn  = master.get_analog(ANALOG_RIGHT_X) * 0.787401574803;
    double turn    = button_turn + drivetoposition::turn + temp_turn;
    // double turn    = button_turn + drivetoposition::turn    + pow(abs(temp_turn / 100), 1.8) * 100 * sgn(temp_turn);
    double sync = std::min(1.0, 100 / (fabs(forward) + fabs(strafe) + fabs(turn)));

    double drive_fl_pct = drive_fl_slew.new_value((forward + strafe + turn) * sync);
    double drive_fr_pct = drive_fr_slew.new_value((forward - strafe - turn) * sync);
    double drive_bl_pct = drive_bl_slew.new_value((forward - strafe + turn) * sync);
    double drive_br_pct = drive_br_slew.new_value((forward + strafe - turn) * sync);

    drive_fl->moveVelocity(drive_fl_pct * 2);
    drive_fr->moveVelocity(drive_fr_pct * 2);
    drive_bl->moveVelocity(drive_bl_pct * 2);
    drive_br->moveVelocity(drive_br_pct * 2);

    pros::delay(5);
  }
}



using namespace controllerbuttons;

Macro goal_turn_right(
    [&](){
      button_strafe = 100;
      button_turn = -70;
      while (true) {
        button_forward = 0.04 * (MIN(goal_sensor_one.get_value(), goal_sensor_two.get_value() - 2300));
        wait(10);
      }
    },
    [](){
        button_strafe = 0;
        button_turn = 0;
        button_forward = 0;
    },
    {&drive_group});

Macro goal_turn_left(
    [&](){
      button_strafe = -100;
      button_turn = 80;
      while (true) {
        button_forward = 0.04 * (MIN(goal_sensor_one.get_value(), goal_sensor_two.get_value() - 2300));
        wait(10);
      }
    },
    [](){
        button_strafe = 0;
        button_turn = 0;
        button_forward = 0;
    },
    {&drive_group});

void drive_group_terminate() {
  drive_group.terminate();
  button_strafe = 0;
  button_turn = 0;
  button_forward = 0;
}

void set_callbacks() {
  using namespace controllerbuttons;
  button_handler.master.left.pressed.set_macro(goal_turn_left);
  button_handler.master.right.pressed.set_macro(goal_turn_right);
  button_handler.master.left.released.set(drive_group_terminate);
  button_handler.master.right.released.set(drive_group_terminate);
}

} // namespace autondrive



namespace autonroutines {
using namespace autondrive;
using namespace drivetoposition;
using namespace odomutilities;
using namespace controllerbuttons;
using namespace robotfunctions;
using namespace rollers;

int start_time = 0;


void auton_log() {
  std::string odom_log = "x,y,theta,balls_in_robot,intake_queue,score_queue,eject_queue,target_distance_reached,target_heading_reached,final_target_reached,targets,goal_sensor_one.get_value(),goal_sensor_two.get_value()";
  std::string odom_log_number_old = "";
  std::string odom_log_number_new = "";

  for (int i = 0; i < 1500; i++) {
    OdomState odom = get_odom_state();
    odom_log.append("\n" +
        std::to_string(odom.x.convert(inch)) + "," +
        std::to_string(odom.y.convert(inch)) + "," +
        std::to_string(odom.theta.convert(degree)) + "," +
        std::to_string(balls_in_robot.size()) + "," +
        std::to_string(intake_queue) + "," +
        std::to_string(score_queue) + "," +
        std::to_string(eject_queue) + "," +
        std::to_string(target_distance_reached) + "," +
        std::to_string(target_heading_reached) + "," +
        std::to_string(final_target_reached) + "," +
        std::to_string(targets.size()) + "," +
        std::to_string(goal_sensor_one.get_value()) + "," +
        std::to_string(goal_sensor_two.get_value()));
    pros::delay(10); 
  }
  
  std::ifstream odl_i("/usd/odom_log_number.txt");
  odl_i >> odom_log_number_old;
  odl_i.close();

  odom_log_number_new = std::to_string(std::stoi(odom_log_number_old) + 1);

  std::ofstream odl_o("/usd/odom_log_number.txt");
  odl_o << odom_log_number_new << std::endl;
  odl_o.close();

  std::ofstream logfile(("/usd/auton_log_" + odom_log_number_new + ".csv").c_str());
  logfile << odom_log << std::endl;
  logfile.close();
}

void auton_init(OdomState odom_state) {
  imu_odom->setState(odom_state);
  start_time = pros::millis();
  auton_drive_enabled = true;
  (pros::Task(auton_log));
}

void auton_clean_up() {
  clear_all_targets();
  auton_drive_enabled = false;
  button_strafe = 0;
  button_turn = 0;
  button_forward = 0;
  controllermenu::master_print_array[0] = "Completed";
  controllermenu::master_print_array[1] = "Time: " + std::to_string(pros::millis() - start_time);
  controllermenu::master_print_array[2] = "";
}

Macro none([&](){},[](){});


Macro test(
    [&](){
      auton_init({13.491_in, 34.9911_in, 0_deg});

      move_settings.start_output = 100;
      move_settings.end_output = 20;

      add_target(18_in, 34.9911_in, 0_deg);
      add_target(goal_1, -135_deg, 25_in);
      drive_to_goal(goal_1, -135_deg);
      add_target(goal_1, -135_deg, 30_in);
      add_target(goal_1, 0_deg, 30_in, -135_deg);
      add_target(13.491_in, 34.9911_in, 0_deg);

      wait_until_final_target_reached();
    },
    [](){
      auton_clean_up();
    },
    {&auton_group});

Macro home_row_three(
    [&](){
      auton_init({15.7416_in, 31.4911_in, -90_deg});

      move_settings.start_output = 100;
      move_settings.end_output = 20;

      intake_queue = 3;
      wait(700);
      drive_to_goal(goal_1, -135_deg); // at goal 1
      score_balls(2); // score
      add_target(goal_1, -135_deg, 29_in); // back away
      wait_until_final_target_reached();
      wait(100);
      intakes_back.start();
      add_target(goal_2, -180_deg, 29_in);
      eject_queue = 1;
      wait(1000);
      eject_all_but(1);

      wait_until_final_target_reached();
      intake_queue = 2;
      drive_to_goal(goal_2, -180_deg); // at goal 2
      score_balls(2); // score
      wait(300);
      add_target(goal_2, -180_deg, 25_in); // back away
      add_target(goal_3, -180_deg, 35_in, -225_deg);
      wait(100);
      eject_all_but(0);
      wait(300);
      intakes_back.start();

      wait_until_final_target_reached();
      add_target(goal_3, -225_deg, 35_in);
      wait(500);
      intake_queue = 3;
      wait(500);
      drive_to_goal(goal_3, -225_deg); // at goal 3
      score_balls(2); // score
      add_target(goal_3, -225_deg, 35_in); // back away


      wait_until_final_target_reached();
    },
    [](){
      auton_clean_up();
    },
    {&auton_group});

Macro home_row_three_old(
    [&](){
      auton_init({15.7416_in, 31.4911_in, -90_deg});

      move_settings.start_output = 100;
      move_settings.end_output = 20;
      
      add_target(26.319_in, 26.319_in, -90_deg);
      add_target(26.319_in, 26.319_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 10;
      wait(400);
      top_roller_smart.add_target(45, 30);
      add_target(20.4_in, 20.4_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      wait(400);
      intake_queue = 0;
      intakes_back.start();
      wait(300);
      add_target(16_in, 16_in, -135_deg);
      wait(10);
      WAIT_UNTIL(final_target_reached)
      add_target(13.6_in, 13.6_in, -135_deg);
      wait(200);
      score_queue = 1;
      wait(200);
      targets.pop();
      // add_target(30_in, 30_in, -135_deg);
      add_target(26_in, 72_in, -180_deg);
      // add_target(22_in, 72_in, -180_deg);

      wait(10);
      WAIT_UNTIL(final_target_reached)
      add_target(17_in, 72_in, -180_deg);
      move_settings.start_output = 20;
      move_settings.end_output = 20;
      wait(900);
      score_queue = 1;
      wait(200);
      targets.pop();
      wait(10);
      // imu_odom->setState({20.75_in, 71.63_in, -171.5_deg});

      move_settings.start_output = 100;
      move_settings.end_output = 50;
      wait(10);
      add_target(32_in, 72_in, -180_deg);
      wait(10);
      WAIT_UNTIL(final_target_reached)
      move_settings.end_output = 20;

      // add_target(32_in, 109_in, -180_deg);
      add_target(32_in, 116_in, -180_deg);
      // wait(500);
      // add_target(32_in, 109_in, -225_deg);
      add_target(32_in, 116_in, -225_deg);
      wait(10);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(500);
      add_target(23_in, 126_in, -225_deg);
      wait(10);
      WAIT_UNTIL(final_target_reached)
      wait(300);
      intake_queue = 0;
      intakes_back.start();
      wait(30);
      intakes_back.terminate();
      wait(270);
      // add_target(13.6_in, 127_in, -225_deg);
      add_target(6_in, 150.85_in, -225_deg);
      wait(500);
      score_queue = 1;
      wait(200);
      targets.pop();
      wait(10);
      // wait(200);
      // add_target(32_in, 109_in, -180_deg);
      add_target(32_in, 118_in, -225_deg);
      wait(10);
      WAIT_UNTIL(final_target_reached)
      score_queue = 0;
    },
    [](){
      auton_clean_up();
    },
    {&auton_group});

Macro home_row_two(
    [&](){
      imu_odom->setState({15.7416_in, 31.4911_in, -90_deg});

      move_settings.start_output = 100;
      move_settings.end_output = 20;
      
      add_target(26.319_in, 26.319_in, -90_deg);
      add_target(26.319_in, 26.319_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 10;
      wait(400);
      top_roller_smart.add_target(45, 30);
      add_target(20.4_in, 20.4_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      wait(400);
      intake_queue = 0;
      intakes_back.start();
      wait(300);
      add_target(16_in, 16_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      add_target(13.6_in, 13.6_in, -135_deg);
      wait(200);
      score_queue = 1;
      wait(200);
      targets.pop();
      // add_target(30_in, 30_in, -135_deg);
      add_target(26_in, 72_in, -180_deg);
      // add_target(22_in, 72_in, -180_deg);

      WAIT_UNTIL(final_target_reached)
      add_target(17_in, 72_in, -180_deg);
      move_settings.start_output = 20;
      move_settings.end_output = 20;
      wait(500);
      score_queue = 1;
      wait(200);
      targets.pop();
      wait(10);
      move_settings.start_output = 100;
      move_settings.end_output = 50;
      add_target(32_in, 72_in, -180_deg);
      WAIT_UNTIL(final_target_reached)
      move_settings.end_output = 20;
      score_queue = 0;
    },
    [](){
      targets_should_clear = true;
    },
    {&auton_group});

Macro left_shawnton(
    [&](){
      imu_odom->setState({15.7416_in, 31.4911_in, -90_deg});

      add_target(26.319_in, 26.319_in, -90_deg);
      add_target(26.319_in, 26.319_in, -135_deg);
      add_target(16_in, 16_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      add_target(13.6_in, 13.6_in, -135_deg);
      intake_left.move_relative(30, 200);
      intake_right.move_relative(30, 200);
      wait(500);
      intake_left.move_relative(-30, 200);
      intake_right.move_relative(-30, 200);
      top_roller_smart.add_target(45, 30);
      wait(500);
      score_queue = 1;
      wait(200);
      targets.pop();
      wait(500);
      add_target(30_in, 30_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
    },
    [](){
      targets_should_clear = true;
    },
    {&auton_group});

Macro right_shawnton(
    [&](){
      imu_odom->setState({31.4911_in, 15.7416_in, -180_deg});

      add_target(26.319_in, 26.319_in, -180_deg);
      WAIT_UNTIL(final_target_reached)
      add_target(26.319_in, 26.319_in, -135_deg);
      wait(500);
      targets.pop();
      add_target(16_in, 16_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
      add_target(13.6_in, 13.6_in, -135_deg);
      intake_left.move_relative(30, 200);
      intake_right.move_relative(30, 200);
      wait(500);
      intake_left.move_relative(-30, 200);
      intake_right.move_relative(-30, 200);
      top_roller_smart.add_target(45, 30);
      wait(500);
      score_queue = 1;
      wait(200);
      targets.pop();
      wait(500);
      add_target(30_in, 30_in, -135_deg);
      WAIT_UNTIL(final_target_reached)
    },
    [](){
      targets_should_clear = true;
    },
    {&auton_group});

void stop_scoring() {
  score_queue = 0;
  top_roller_smart.target_queue = {};
  // top_roller_smart.set_manual_speed(0, 0);
  // top_roller_smart.set_manual_speed(1, 0);
  top_roller_smart.auto_speed = 0;
}

Macro skills_one(
    [&](){
      imu_odom->setState({13.491_in, 34.9911_in, 0_deg});
      using namespace skillsballs;

      move_settings.start_output = 100;
      move_settings.end_output = 20;

      intake_queue = 1;
      add_target(18_in, 34.9911_in, 0_deg);
      add_target(5.8129_in, 5.8129_in, -135_deg, 17_in);
      wait(500);
      intakes_back.start();
      WAIT_UNTIL(final_target_reached)
      add_target(5.8129_in, 5.8129_in, -135_deg, 6_in);
      wait(500);
      score_queue = 1;
      wait(300);
      targets.pop();
      stop_scoring();

      add_target(ball_e, -300_deg, 14_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(700);
      intakes_back.start();

      add_target(5.9272_in, 70.3361_in, -180_deg, 20_in);
      WAIT_UNTIL(final_target_reached)
      add_target(5.9272_in, 70.3361_in, -180_deg, 6_in);
      wait(600);
      score_queue = 2;
      wait(1000);
      targets.pop();

      stop_scoring();
      // move_settings.end_output = 100;
      add_target(23_in, 70.3361_in, -180_deg);
      add_target(23_in, 90_in, -270_deg);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      add_target(23_in, 117.18_in, -270_deg);
      // move_settings.end_output = 20;
      WAIT_UNTIL(final_target_reached)
      intakes_back.start();
      add_target(5.8129_in, 134.8593_in, -225_deg, 24.5_in);
      WAIT_UNTIL(final_target_reached)
      add_target(5.8129_in, 134.8593_in, -225_deg, 6_in);
      wait(1000);
      score_queue = 1;
      wait(400);
      targets.pop();

      stop_scoring();
      add_target(23_in, 117.18_in, -270_deg);
      WAIT_UNTIL(final_target_reached)
      // intake_right.move_relative(180, 200);
      add_target(34.8361_in, 123.6722_in, -270_deg);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(1000);
      intakes_back.start();
      add_target(70.3361_in, 117.4624_in, -360_deg, 12_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      add_target(70.3361_in, 117.4624_in, -360_deg);
      wait(1000);
      intakes_back.start();
      add_target(70.3361_in, 117.4624_in, -270_deg);
      WAIT_UNTIL(final_target_reached)
      add_target(70.3361_in, 134.745_in, -270_deg, 6_in);
      wait(500);
      score_queue = 2;
      wait(700);
      targets.pop();

      stop_scoring();
      add_target(70.3361_in, 117.4624_in, -270_deg);
      WAIT_UNTIL(final_target_reached)
      // intake_right.move_relative(180, 200);
      add_target(105.8361_in, 123.6722_in, -270_deg);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(1000);
      intakes_back.start();
      add_target(134.8593_in, 134.8593_in, -315_deg, 17_in);
      WAIT_UNTIL(final_target_reached)
      add_target(134.8593_in, 134.8593_in, -315_deg, 6_in);
      wait(300);
      score_queue = 1;
      wait(300);
      targets.pop();

      stop_scoring();
      add_target(118_in, 125_in, -360_deg);
      add_target(118_in, 125_in, -450_deg);
      add_target(117.6361_in, 105.6361_in, -450_deg, 13_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(1000);
      intakes_back.start();

      add_target(ball_j, -480_deg, 14_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(700);
      intakes_back.start();
      add_target(117.6361_in, 70.3361_in, -450_deg);

      add_target(117.6361_in, 70.3361_in, -360_deg);
      WAIT_UNTIL(final_target_reached)
      add_target(134.745_in,  70.3361_in, -360_deg, 6_in);
      wait(1000);
      score_queue = 2;
      wait(1000);
      targets.pop();

      stop_scoring();
      add_target(117.6361_in, 70.3361_in, -360_deg);
      add_target(117.6361_in, 35.0361_in, -450_deg, 13_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      add_target(117.1816_in, 23.4906_in, -450_deg);
      WAIT_UNTIL(final_target_reached)
      intakes_back.start();
      add_target(134.8593_in, 5.8129_in, -405_deg, 24.5_in);
      WAIT_UNTIL(final_target_reached)
      add_target(134.8593_in, 5.8129_in, -405_deg, 6_in);
      wait(700);
      score_queue = 1;
      wait(400);
      targets.pop();

      stop_scoring();
      add_target(117.1816_in, 23.4906_in, -450_deg);
      WAIT_UNTIL(final_target_reached)
      // intake_right.move_relative(180, 200);
      add_target(106.8361_in, 3.3361_in, -450_deg , 13_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(1000);
      intakes_back.start();
      add_target(70.3361_in, 23.3361_in, -540_deg, 13_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      add_target(70.3361_in, 23.3361_in, -540_deg);
      wait(1000);
      intakes_back.start();
      wait(300);
      add_target(70.3361_in, 23.3361_in, -450_deg);
      WAIT_UNTIL(final_target_reached)
      add_target(70.3361_in, 5.9272_in, -450_deg, 6_in);
      wait(600);
      score_queue = 2;
      wait(1700);
      targets.pop();

      stop_scoring();
      add_target(70.3361_in, 23.3361_in, -450_deg);
      add_target(70.3361_in, 23.3361_in, -270_deg);
      add_target(70.3361_in, 46.8361_in, -270_deg, 14_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(1000);
      add_target(70.3361_in, 70.3361_in, -270_deg, 16_in);
      WAIT_UNTIL(final_target_reached)
      add_target(70.3361_in, 70.3361_in, -270_deg, 6_in);
      wait(300);
      intake_queue = 50;
      wait(200);
      targets.pop();

      button_strafe = 10;
      button_turn = -6.84;
      button_forward = 3;
      wait(1000);
      button_strafe = -10;
      button_turn = 6.84;
      button_forward = 3;
      wait(2000);
      intake_queue = 1;
      score_queue = 1;
      button_strafe = 10;
      button_turn = -6.84;
      button_forward = 3;
      // wait(4000);
      WAIT_UNTIL(intake_queue == 0)
      button_strafe = 0;
      button_turn = 0;
      button_forward = 0;

      intakes_back.start();
    },
    [](){
      button_strafe = 0;
      button_turn = 0;
      button_forward = 0;
      targets_should_clear = true;
    },
    {&auton_group});

Macro skills_two(
    [&](){
      using namespace skillsballs;

      imu_odom->setState({13.491_in, 34.9911_in, 0_deg});

      move_settings.start_output = 100;
      move_settings.end_output = 20;

      intake_queue = 1;
      add_target(34.9911_in, 34.9911_in, 0_deg);
      wait_until_final_target_reached();
      intake_queue = 1;
      add_target(ball_c, -90_deg, 12_in);
      add_target(goal_1, -135_deg, 17_in);
      intake_queue = 2;
      drive_to_goal(goal_1, -135_deg);
      score_balls(1);

      add_target(goal_1, -135_deg, 20_in);
      add_target(goal_1, -135_deg, 20_in);

      add_target(goal_2, -180_deg, 20_in);
      // stop_scoring();
      // add_target(5.9272_in, 70.3361_in, -180_deg, 20_in);
      // wait_until_final_target_reached();
      // add_target(5.9272_in, 70.3361_in, -180_deg, 6_in);
      // wait(600);
      // score_queue = 1;
      // wait(400);
      // targets.pop();

      // stop_scoring();
      // // move_settings.end_output = 100;
      // add_target(23_in, 70.3361_in, -180_deg);
      // add_target(23_in, 90_in, -270_deg);
      // wait_until_final_target_reached();
      // intake_queue = 1;
      // add_target(23_in, 117.18_in, -270_deg);
      // // move_settings.end_output = 20;
      // wait_until_final_target_reached();
      // intakes_back.start();
      // add_target(5.8129_in, 134.8593_in, -225_deg, 24.5_in);
      // wait_until_final_target_reached();
      // add_target(5.8129_in, 134.8593_in, -225_deg, 6_in);
      // wait(1000);
      // score_queue = 1;
      // wait(400);
      // targets.pop();

      // stop_scoring();
      // add_target(23_in, 117.18_in, -270_deg);
      // wait_until_final_target_reached();
      // intake_right.move_relative(180, 200);
      // add_target(34.8361_in, 123.6722_in, -270_deg);
      // wait_until_final_target_reached();
      // intake_queue = 1;
      // wait(1000);
      // intakes_back.start();
      // add_target(70.3361_in, 117.4624_in, -360_deg, 12_in);
      // wait_until_final_target_reached();
      // intake_queue = 1;
      // add_target(70.3361_in, 117.4624_in, -360_deg);
      // wait(1000);
      // intakes_back.start();
      // add_target(70.3361_in, 117.4624_in, -270_deg);
      // wait_until_final_target_reached();
      // add_target(70.3361_in, 134.745_in, -270_deg, 6_in);
      // wait(500);
      // score_queue = 2;
      // wait(700);
      // targets.pop();

      // stop_scoring();
      // add_target(70.3361_in, 117.4624_in, -270_deg);
      // wait_until_final_target_reached();
      // intake_right.move_relative(180, 200);
      // add_target(105.8361_in, 123.6722_in, -270_deg);
      // wait_until_final_target_reached();
      // intake_queue = 1;
      // wait(1000);
      // intakes_back.start();
      // add_target(134.8593_in, 134.8593_in, -315_deg, 17_in);
      // wait_until_final_target_reached();
      // add_target(134.8593_in, 134.8593_in, -315_deg, 6_in);
      // wait(300);
      // score_queue = 1;
      // wait(300);
      // targets.pop();

      // stop_scoring();
      // add_target(118_in, 125_in, -360_deg);
      // add_target(118_in, 125_in, -450_deg);
      // add_target(117.6361_in, 105.6361_in, -450_deg, 12_in);
      // wait_until_final_target_reached();
      // intake_queue = 1;
      // add_target(117.6361_in, 70.3361_in, -450_deg);
      // wait(1000);
      // intakes_back.start();
      // add_target(117.6361_in, 70.3361_in, -360_deg);
      // wait_until_final_target_reached();
      // add_target(134.745_in,  70.3361_in, -360_deg, 6_in);
      // wait(1000);
      // score_queue = 1;
      // wait(200);
      // targets.pop();

      // stop_scoring();
      // add_target(117.6361_in, 70.3361_in, -360_deg);
      // add_target(117.6361_in, 35.0361_in, -450_deg, 12_in);
      // wait_until_final_target_reached();
      // intake_queue = 1;
      // add_target(117.1816_in, 23.4906_in, -450_deg);
      // wait_until_final_target_reached();
      // intakes_back.start();
      // add_target(134.8593_in, 5.8129_in, -405_deg, 24.5_in);
      // wait_until_final_target_reached();
      // add_target(134.8593_in, 5.8129_in, -405_deg, 6_in);
      // wait(700);
      // score_queue = 1;
      // wait(400);
      // targets.pop();

      // stop_scoring();
      // add_target(117.1816_in, 23.4906_in, -450_deg);
      // wait_until_final_target_reached();
      // intake_right.move_relative(180, 200);
      // add_target(105.8361_in, 3.3361_in, -450_deg , 13_in);
      // wait_until_final_target_reached();
      // intake_queue = 1;
      // wait(1000);
      // intakes_back.start();
      // add_target(70.3361_in, 23.3361_in, -540_deg, 11_in);
      // wait_until_final_target_reached();
      // intake_queue = 1;
      // add_target(70.3361_in, 23.3361_in, -540_deg);
      // wait(1000);
      // intakes_back.start();
      // add_target(70.3361_in, 23.3361_in, -450_deg);
      // wait_until_final_target_reached();
      // add_target(70.3361_in, 5.9272_in, -450_deg, 6_in);
      // wait(600);
      // score_queue = 2;
      // wait(1700);
      // targets.pop();

      // stop_scoring();
      // add_target(70.3361_in, 23.3361_in, -450_deg);
      // add_target(70.3361_in, 23.3361_in, -270_deg);
      // add_target(70.3361_in, 46.8361_in, -270_deg, 10_in);
      // wait_until_final_target_reached();
      // intake_queue = 1;
      // wait(1000);
      // add_target(70.3361_in, 70.3361_in, -270_deg, 16_in);
      // wait_until_final_target_reached();
      // add_target(70.3361_in, 70.3361_in, -270_deg, 6_in);
      // wait(300);
      // intake_queue = 50;
      // wait(200);
      // targets.pop();

      // button_strafe = 10;
      // button_turn = -6.84;
      // button_forward = 3;
      // wait(2000);
      // button_strafe = -10;
      // button_turn = 6.84;
      // button_forward = 3;
      // wait(3000);
      // intake_queue = 1;
      // score_queue = 1;
      // button_strafe = 10;
      // button_turn = -6.84;
      // button_forward = 3;
      // // wait(4000);
      // WAIT_UNTIL(intake_queue == 0)
      // button_strafe = 0;
      // button_turn = 0;
      // button_forward = 0;

      // intakes_back.start();
    },
    [](){
      button_strafe = 0;
      button_turn = 0;
      button_forward = 0;
      targets_should_clear = true;
    },
    {&auton_group});

Macro shawnton_three(
    [&](){
      imu_odom->setState({15.7416_in, 109.181_in, 90_deg});

      add_target(goal_3, 135_deg, 22_in);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 2;
      wait(700);
      drive_to_goal(goal_3, 135_deg);
      score_balls(1);


      add_target(5.8129_in, 134.8593_in, 135_deg, 20_in);
      WAIT_UNTIL(final_target_reached)
      add_target(goal_6, 0_deg, 20_in, 40_deg);
      add_target(70.3361_in, 130.7450_in, 25_deg, 20_in);
      wait(300);
      intake_queue = 1;

      WAIT_UNTIL(final_target_reached)
      add_target(70.3361_in, 132.7450_in, 25_deg, 6_in, 25_deg);
      wait(1000);
      clear_all_targets();

      add_target(70.3361_in, 134.745_in, 40_deg, 20_in);
      WAIT_UNTIL(final_target_reached)
      intakes_back.start();
      wait(300);
      drive_to_goal(goal_6, 30_deg);
      score_balls(1);
      add_target(goal_6, 30_deg, 20_in);
      // add_target(70.3361_in, 134.745_in, 30_deg, 20_in);

      add_target(goal_5, 0_deg, 24_in);
      WAIT_UNTIL(final_target_reached)
      drive_to_goal(goal_5, 0_deg);
      // wait(1200);
      score_balls(1);
      intake_left.move_relative(50, 200);
      intake_right.move_relative(50, 200);
      wait(800);
      intake_left.move_relative(-50, 200);
      intake_right.move_relative(-50, 200);
      // wait(300);
      // targets.pop();
      add_target(70.3361_in, 70.3361_in, 0_deg, 16_in);
      WAIT_UNTIL(final_target_reached)
    },
    [](){
      targets_should_clear = true;
    },

    {&auton_group});

Macro shawnton_cycle(
    [&](){
      imu_odom->setState({15.7416_in, 109.181_in, 90_deg});

      add_target(23.49_in, 117.18_in, 90_deg);
      add_target(5.8129_in, 134.8593_in, 135_deg, 25_in);
      add_target(5.8129_in, 134.8593_in, 135_deg, 16_in);
      // add_target(5.8129_in, 134.8593_in, 135_deg, 16_in);
      wait(10);
      WAIT_UNTIL(final_target_reached)
      intake_queue = 1;
      wait(800);
      intake_left = 0;
      intake_right = 0;
      intake_left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      intake_right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      add_target(5.8129_in, 134.8593_in, 135_deg, 25_in);
      add_target(5.8129_in, 134.8593_in, 135_deg, 16_in);
      wait(10);
      WAIT_UNTIL(final_target_reached)
      add_target(5.8129_in, 134.8593_in, 135_deg, 6_in);
      wait(300);
      score_queue = 2;
      wait(1000);
      targets.pop();

      intake_queue = 1;
      wait(800);
      intake_left = 0;
      intake_right = 0;
      intake_left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      intake_right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      intake_queue = 1;
      wait(800);
      intake_left = 0;
      intake_right = 0;
      intake_left.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      intake_right.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
      score_queue = 1;
      intakes_back.start();
      wait(200);

      add_target(5.8129_in, 134.8593_in, 135_deg, 20_in);
      wait(10);
      
      WAIT_UNTIL(final_target_reached)
    },
    [](){
      targets_should_clear = true;
    },
    {&auton_group});

} // namespace autonroutines