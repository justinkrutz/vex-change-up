#include "api.h"

#include <bits/stdc++.h>

#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"

namespace controllermenu {
controllerbuttons::MacroGroup menu;

enum MenuItemType {kFolder, kAutonomous, kSettingSwitch, kSettingSlider, kAction};

const char *item_type_name(int item) {
  switch (item) {
  case kFolder:
    return "Folder";
  case kAutonomous:
    return "Autonomous";
  case kSettingSwitch:
    return "Setting";
  case kSettingSlider:
    return "Setting";
  case kAction:
    return "Action";
  }
  return 0;
}

int current_item(0);

struct item_struct {
  int parent_item_number;
  MenuItemType item_type;
  std::vector<int> items;
  const char *name;
  const char *description;
  void (*function)();
  int setting_default;

  int setting_value;
  int cursor_location;
};
// 19 or less chars!!!
// 1234567890123456789
std::vector <item_struct> database = {
  ////// parent, ----type-----, ----------items--------, --------"name"-------
  ////////   ---"descrtiption"---, -----------function----------, ------"name"------
  /*  0 */  { 0, kFolder,        {1,  2,  3,  4,  5, 11},     "kFolder is Empty"},
  /*  1 */  { 0, kFolder,        {6                    },                 "Red"},
  /*  2 */  { 0, kFolder,        {7,  7,  7,  7,  7,  7},                "Blue"},
  /*  3 */  { 0, kFolder,        {8                    },              "Skills"},
  /*  4 */  { 0, kFolder,        {0                    },               "Other"},
  /*  5 */  { 0, kFolder,        {9, 10                },            "Settings"},
  /*  6 */  { 1, kAutonomous,    {0},             "Red one", "1234567890123456789", robotfunctions::count_down_task},
  /*  7 */  { 2, kAutonomous,    {0},            "Blue one",                  "Scores points", robotfunctions::count_down_task},
  /*  8 */  { 3, kAutonomous,    {0},          "Skills one",                  "Scores points", robotfunctions::count_down_task},
  /*  9 */  { 5, kSettingSlider, {0},              "Slider",             "", NULL, 57},
  /* 10 */  { 5, kSettingSwitch, {0},              "Switch",             "", NULL, 1},
  /* 11 */  { 0, kFolder,        {12, 13, 14           },             "Actions"},
  // /* 12 */  {11, kAction,        {0},      "Reset defaults",             "", reset_setting_defaults},
  // /* 13 */  {11, kAction,        {0},      "Store Settings",             "", store_settings},
  // /* 14 */  {11, kAction,        {0},       "Load Settings",             "", load_settings},
  /* 15 */
  /* 16 */
  /* 17 */
  /* 18 */
};


const char *int_to_const_char_p(int input) {
  std::stringstream str;
  str << input;
  const char *output = str.str().c_str();
  return output;
}

void print_folder() {
  std::string selection = "[_][_][_][_][_][_]";
  selection.resize(database[current_item].items.size() * 3);
  selection.replace((database[current_item].cursor_location) * 3 + 1, 1, "o");

  master.clear();
  pros::delay(50);
  master.print(0, 0, selection.c_str());
  master.print(1, 0, item_type_name(database[database[current_item].items
                           [database[current_item].cursor_location]].item_type));
  master.print(2, 0, database[database[current_item].items
                           [database[current_item].cursor_location]].name);
}

void print_auton() {
  master.clear();
  pros::delay(50);
  master.print(0, 0, database[current_item].name);
}

void print_setting_slider() {
  std::string bar = "l.................l";
  int barValue = database[current_item].setting_value * 0.17;
  bar.replace(1, barValue, barValue, '!');

  master.clear();
  pros::delay(50);
  master.print(0, 0, database[current_item].name);
  master.print(1, 0, bar.c_str());
  master.print(2, 0, "%d%%", database[current_item].setting_value);
}

void print_setting_switch() {
  master.clear();
  pros::delay(50);
  master.print(0, 0, database[current_item].name);
  if (database[current_item].setting_value) {
    master.print(1, 0, "True");
  } else {
    master.print(1, 0, "False");
  }
}

void print_action(const char *message) {
  master.clear();
  pros::delay(50);
  master.print(0, 0, database[current_item].name);
  master.print(1, 0, database[current_item].description);
  master.print(2, 0, message);
}

void scroll(int direction) {
  switch (database[current_item].item_type) {
    int tempCursorLocation;
    int tempSettingValue;
  case kFolder:
    tempCursorLocation = database[current_item].cursor_location + direction;
    if (tempCursorLocation < 0) {
      tempCursorLocation = database[current_item].items.size() - 1;
    } else if (tempCursorLocation > database[current_item].items.size() - 1) {
      tempCursorLocation = 0;
    }

    database[current_item].cursor_location = tempCursorLocation;
    break;
  case kSettingSlider:
    tempSettingValue = database[current_item].setting_value + direction;
    if (database[current_item].setting_value + direction < 0) {
      tempSettingValue = 0;
    } else if (database[current_item].setting_value + direction > 100) {
      tempSettingValue = 100;
    }
    database[current_item].setting_value = tempSettingValue;
    break;
  default:
    break;
  }
  print_menu();
}

void scrollRight() { scroll(1); }

void scrollLeft() { scroll(-1); }

void scrollUp() { scroll(10); }

void scrollDown() { scroll(-10); }

void back() {
  current_item = database[current_item].parent_item_number;
  print_menu();
}

void select() {
  switch (database[current_item].item_type) {
  case kFolder:
    current_item = database[current_item].items[database[current_item].cursor_location];
    print_menu();
    break;
  case kSettingSwitch:
    database[current_item].setting_value = !database[current_item].setting_value;
    print_menu();
    break;
  case kAction:
    print_action("Running...");
    database[current_item].function();
    print_action("Complete!");
    master.rumble(".");
    break;
  default:
    break;
  }
}

void print_menu() {
  switch (database[current_item].item_type) {
  case kFolder:
    print_folder();
    break;
  case kAutonomous:
    print_auton();
    break;
  case kSettingSwitch:
    print_setting_switch();
    break;
  case kSettingSlider:
    print_setting_slider();
    break;
  case kAction:
    print_action("Press 'A' to run");
    break;
  }
}

void set_callbacks() {
  using namespace controllerbuttons;
  button_callbacks = {
    {&master, BTN_RIGHT, false, {&menu}, &scrollRight},
    {&master, BTN_LEFT,  false, {&menu}, &scrollLeft},
    {&master, BTN_UP,    false, {&menu}, &scrollUp},
    {&master, BTN_DOWN,  false, {&menu}, &scrollDown},
    {&master, BTN_A,     false, {&menu}, &select},
    {&master, BTN_B,     false, {&menu}, &back},
  };
}

void run_auton() {
  if (database[current_item].item_type == kAutonomous) {
    master.rumble(".");
    database[current_item].function();
  }
}

void check_for_auton() {
  if (database[current_item].item_type != kAutonomous) {
    master.rumble(".");
  }
}

// void reset_setting_defaults() {
//   for (int i = 0; i < database.size(); i++) {
//     if (database[i].item_type == kSettingSlider ||
//         database[i].item_type == kSettingSwitch) {
//       uint8_t data_in = database[i].setting_default;
//       Brain.SDcard.savefile(int_to_const_char_p(i), &data_in, 3);
//     }
//   }
// }
//
// void load_settings() {
//   for (int i = 0; i < database.size(); i++) {
//     if (database[i].item_type == kSettingSlider ||
//         database[i].item_type == kSettingSwitch) {
//       uint8_t data_in;
//       Brain.SDcard.loadfile(int_to_const_char_p(i), &data_in, 3);
//       int dataOut = data_in;
//       database[i].setting_value = dataOut;
//     }
//   }
// }
//
// void store_settings() {
//   for (int i = 0; i < database.size(); i++) {
//     if (database[i].item_type == kSettingSlider ||
//         database[i].item_type == kSettingSwitch) {
//       uint8_t data_in = database[i].setting_value;
//       Brain.SDcard.savefile(int_to_const_char_p(i), &data_in, 3);
//     }
//   }
// }

} // namespace controllermenu
