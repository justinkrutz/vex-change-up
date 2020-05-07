// #include "api.h"
//
// #include <bits/stdc++.h>
//
// #include "robot-config.h"
// #include "controller-buttons.h"
// #include "controller-menu.h"
// #include "robot-functions.h"
//
// namespace controllermenu {
// controllerbuttons::MacroGroup menu;
//
// enum MenuItemType {kFolder, kAutonomous, kSettingSwitch, kSettingSlider, kAction};
//
// const char *item_type_name(int item) {
//   switch (item) {
//   case kFolder:
//     return "Folder";
//   case kAutonomous:
//     return "Autonomous";
//   case kSettingSwitch:
//     return "Setting";
//   case kSettingSlider:
//     return "Setting";
//   case kAction:
//     return "Action";
//   }
//   return 0;
// }
//
// int current_item(0);
//
// struct item_struct {
//   int parent_item_number;
//   MenuItemType item_type;
//   std::vector<int> items;
//   const char *name;
//   const char *description;
//   void (*function)();
//   int setting_default;
//
//   int setting_value;
//   int cursor_location;
// };
// // 19 or less chars!!!
// // 1234567890123456789
// std::vector <item_struct> database = {
//   ////// parent, ----type-----, ----------items--------, --------"name"-------
//   ////////   ---"descrtiption"---, -----------function----------, ------"name"------
//   /*  0 */  { 0, kFolder,        {1,  2,  3,  4,  5, 11},     "kFolder is Empty"},
//   /*  1 */  { 0, kFolder,        {6                    },                 "Red"},
//   /*  2 */  { 0, kFolder,        {7,  7,  7,  7,  7,  7},                "Blue"},
//   /*  3 */  { 0, kFolder,        {8                    },              "Skills"},
//   /*  4 */  { 0, kFolder,        {0                    },               "Other"},
//   /*  5 */  { 0, kFolder,        {9, 10                },            "Settings"},
//   /*  6 */  { 1, kAutonomous,    {0},             "Red one", "1234567890123456789", robotfunctions::countDownTask},
//   /*  7 */  { 2, kAutonomous,    {0},            "Blue one",                  "Scores points", robotfunctions::countDownTask},
//   /*  8 */  { 3, kAutonomous,    {0},          "Skills one",                  "Scores points", robotfunctions::countDownTask},
//   /*  9 */  { 5, kSettingSlider, {0},              "Slider",             "", NULL, 57},
//   /* 10 */  { 5, kSettingSwitch, {0},              "Switch",             "", NULL, 1},
//   /* 11 */  { 0, kFolder,        {12, 13, 14           },             "Actions"},
//   /* 12 */  {11, kAction,        {0},      "Reset defaults",             "", resetSettingDefaults},
//   /* 13 */  {11, kAction,        {0},      "Store Settings",             "", storeSettings},
//   /* 14 */  {11, kAction,        {0},       "Load Settings",             "", loadSettings},
//   /* 15 */
//   /* 16 */
//   /* 17 */
//   /* 18 */
// };
//
//
// const char *int_to_const_char_p(int input) {
//   std::stringstream str;
//   str << input;
//   const char *output = str.str().c_str();
//   return output;
// }
//
// void printFolder() {
//   std::string selection = "[_][_][_][_][_][_]";
//   selection.resize(database[current_item].items.size() * 3);
//   selection.replace((database[current_item].cursor_location) * 3 + 1, 1, "o");
//
//   Controller1.Screen.clearScreen();
//   Controller1.Screen.setCursor(1, 0);
//   Controller1.Screen.print(selection.c_str());
//   Controller1.Screen.setCursor(2, 0);
//   Controller1.Screen.print(item_type_name(database[database[current_item].items
//                            [database[current_item].cursor_location]].item_type));
//   Controller1.Screen.setCursor(3, 0);
//   Controller1.Screen.print(database[database[current_item].items
//                            [database[current_item].cursor_location]].name);
// }
//
// void printAuton() {
//   Controller1.Screen.clearScreen();
//   Controller1.Screen.setCursor(1, 0);
//   Controller1.Screen.print(database[current_item].name);
// }
//
// void printSettingSlider() {
//   std::string bar = "l.................l";
//   int barValue = database[current_item].setting_value * 0.17;
//   bar.replace(1, barValue, barValue, '!');
//
//   Controller1.Screen.clearScreen();
//   Controller1.Screen.setCursor(1, 0);
//   Controller1.Screen.print(database[current_item].name);
//   Controller1.Screen.setCursor(2, 0);
//   Controller1.Screen.print(bar.c_str());
//   Controller1.Screen.setCursor(3, 0);
//   Controller1.Screen.print("%d%%", database[current_item].setting_value);
// }
//
// void printSettingSwitch() {
//   Controller1.Screen.clearScreen();
//   Controller1.Screen.setCursor(1, 0);
//   Controller1.Screen.print(database[current_item].name);
//   Controller1.Screen.setCursor(2, 0);
//   if (database[current_item].setting_value) {
//     Controller1.Screen.print("True");
//   } else {
//     Controller1.Screen.print("False");
//   }
// }
//
// void printAction(const char *message) {
//   Controller1.Screen.clearScreen();
//   Controller1.Screen.setCursor(1, 0);
//   Controller1.Screen.print(database[current_item].name);
//   Controller1.Screen.setCursor(2, 0);
//   Controller1.Screen.print(database[current_item].description);
//   Controller1.Screen.setCursor(3, 0);
//   Controller1.Screen.print(message);
// }
//
// void scroll(int direction) {
//   switch (database[current_item].item_type) {
//     int tempCursorLocation;
//     int tempSettingValue;
//   case kFolder:
//     tempCursorLocation = database[current_item].cursor_location + direction;
//     if (tempCursorLocation < 0) {
//       tempCursorLocation = database[current_item].items.size() - 1;
//     } else if (tempCursorLocation > database[current_item].items.size() - 1) {
//       tempCursorLocation = 0;
//     }
//
//     database[current_item].cursor_location = tempCursorLocation;
//     break;
//   case kSettingSlider:
//     tempSettingValue = database[current_item].setting_value + direction;
//     if (database[current_item].setting_value + direction < 0) {
//       tempSettingValue = 0;
//     } else if (database[current_item].setting_value + direction > 100) {
//       tempSettingValue = 100;
//     }
//     database[current_item].setting_value = tempSettingValue;
//     break;
//   default:
//     break;
//   }
//   printMenu();
// }
//
// void scrollRight() { scroll(1); }
//
// void scrollLeft() { scroll(-1); }
//
// void scrollUp() { scroll(10); }
//
// void scrollDown() { scroll(-10); }
//
// void back() {
//   current_item = database[current_item].parent_item_number;
//   printMenu();
// }
//
// void select() {
//   switch (database[current_item].item_type) {
//   case kFolder:
//     current_item = database[current_item].items[database[current_item].cursor_location];
//     printMenu();
//     break;
//   case kSettingSwitch:
//     database[current_item].setting_value = !database[current_item].setting_value;
//     printMenu();
//     break;
//   case kAction:
//     printAction("Running...");
//     database[current_item].function();
//     printAction("Complete!");
//     Controller1.rumble(".");
//     break;
//   default:
//     break;
//   }
// }
//
// void printMenu() {
//   switch (database[current_item].item_type) {
//   case kFolder:
//     printFolder();
//     break;
//   case kAutonomous:
//     printAuton();
//     break;
//   case kSettingSwitch:
//     printSettingSwitch();
//     break;
//   case kSettingSlider:
//     printSettingSlider();
//     break;
//   case kAction:
//     printAction("Press 'A' to run");
//     break;
//   }
// }
//
// void setCallbacks() {
//   using namespace controllerbuttons;
//   button_callbacks = {
//     {&Controller1.ButtonRight, false, {&menu}, &scrollRight},
//     {&Controller1.ButtonLeft,  false, {&menu}, &scrollLeft},
//     {&Controller1.ButtonUp,    false, {&menu}, &scrollUp},
//     {&Controller1.ButtonDown,  false, {&menu}, &scrollDown},
//     {&Controller1.ButtonA,     false, {&menu}, &select},
//     {&Controller1.ButtonB,     false, {&menu}, &back},
//   };
// }
//
// void runAuton() {
//   if (database[current_item].item_type == kAutonomous) {
//     Controller1.rumble(".");
//     database[current_item].function();
//   }
// }
//
// void checkForAuton() {
//   if (database[current_item].item_type != kAutonomous) {
//     Controller1.rumble(".");
//   }
// }
//
// void resetSettingDefaults() {
//   for (int i = 0; i < database.size(); i++) {
//     if (database[i].item_type == kSettingSlider ||
//         database[i].item_type == kSettingSwitch) {
//       uint8_t data_in = database[i].setting_default;
//       Brain.SDcard.savefile(int_to_const_char_p(i), &data_in, 3);
//     }
//   }
// }
//
// void loadSettings() {
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
// void storeSettings() {
//   for (int i = 0; i < database.size(); i++) {
//     if (database[i].item_type == kSettingSlider ||
//         database[i].item_type == kSettingSwitch) {
//       uint8_t data_in = database[i].setting_value;
//       Brain.SDcard.savefile(int_to_const_char_p(i), &data_in, 3);
//     }
//   }
// }
//
// } // namespace controllermenu
