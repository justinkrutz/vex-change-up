#include "api.h"

#include <bits/stdc++.h>

#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"

namespace controllermenu {

controllerbuttons::MacroGroup menu;

std::string controller_print_array [3];

void controller_print() {
  std::string array_last [3];
  while (true) {
    for (int i = 0; i < 3; i++) {
      // printf("controller_print_array[%d]: %s\n", i, controller_print_array[i].c_str());
      if (controller_print_array[i] != array_last[i]) {
        array_last[i] = controller_print_array[i];
        char print_str[20];
        sprintf(print_str, "%-19s", controller_print_array[i].c_str());
        master.print(i, 0, print_str);
        pros::delay(50);
      }
    }
    pros::delay(10);
  }
}

enum MenuItemType {kFolder, kAutonomous, kSettingSwitch, kSettingSlider, kAction};

std::map<const int, const char*> MenuItemTypeName = {
  {kFolder,        "Folder"},
  {kAutonomous,    "Autonomous"},
  {kSettingSwitch, "SettingSwitch"},
  {kSettingSlider, "SettingSlider"},
  {kAction,        "Action"}
};

class MenuItem {
  private:

  public:
    virtual void print() {
      printf("virtual void print()\n");
    }
    virtual void set_callbacks() {
      printf("virtual void set_callbacks()\n");
    }
    // virtual void select(MenuItem ** item_ptr) {}
    // virtual void back(MenuItem ** item_ptr) {}
    // virtual void scroll_right() {}
    // virtual void scroll_left() {}
    // virtual void scroll_up() {}
    // virtual void scroll_down() {}
    MenuItemType item_type;
    const char *name;
};


MenuItem *current_item;

class Folder : public MenuItem {
  private:
    int cursor_location = 0;
    void scroll(int direction) {
      printf("scroll\n");
      int temp_cursor_location;
      int temp_setting_value;
      temp_cursor_location = cursor_location + direction;
      if (temp_cursor_location < 0) {
        temp_cursor_location = children.size() - 1;
      } else if (temp_cursor_location > children.size() - 1) {
        temp_cursor_location = 0;
      }

      cursor_location = temp_cursor_location;
      print();
    }

  public:
    std::vector<MenuItem*> children;
    Folder(const char *name_arg, std::vector<MenuItem*> children_arg) {
      name = name_arg;
      children = children_arg;
      item_type = kFolder;
    }
    // const char *name;

    void set_callbacks() {
      printf("Folder::set_callbacks()\n");
      // using namespace controllerbuttons;
      // button_callbacks = {
      //   {&master, BTN_RIGHT, false, {&menu}, std::bind(&Folder::scroll_right, this)},
      //   {&master, BTN_LEFT,  false, {&menu}, std::bind(&Folder::scroll_left, this)},
      //   {&master, BTN_UP,    false, {&menu}, std::bind(&Folder::scroll_up, this)},
      //   {&master, BTN_DOWN,  false, {&menu}, std::bind(&Folder::scroll_down, this)},
      //   {&master, BTN_A,     false, {&menu}, std::bind(&Folder::select, this)},
      //   {&master, BTN_B,     false, {&menu}, std::bind(&Folder::back, this)},
      // };
    }

    void print() {
      // set_callbacks();
      printf("print_folder\n");
      // Folder * item_to_print = (Folder*)item;
      std::string selection = "[_][_][_][_][_][_]";
      selection.resize(children.size() * 3);
      printf("cursor_location: %d\n", cursor_location);
      selection.replace((cursor_location) * 3 + 1, 1, "o");

      controller_print_array[0] = selection;
      controller_print_array[1] = "Type goes here";
      controller_print_array[1] = MenuItemTypeName.at(children[cursor_location]->item_type);
      controller_print_array[2] = children[cursor_location]->name;
    }

    void select() {
      printf("select\n");
      if (children.size() > 0) {
        printf("this: %p\n", this);
        printf("&children[cursor_location]: %p\n", children[cursor_location]);
        printf("cursor_location: %d\n", cursor_location);
        printf("children[cursor_location]->name: %s\n", children[cursor_location]->name);
        printf(" children.size(): %d\n", children.size());
        // children[cursor_location]->print();
        printf("current_item before: %p\n", current_item);
        current_item = children[cursor_location];
        printf("current_item->name: %s\n", current_item->name);
        printf("current_item after: %p\n", current_item);
        pros::delay(1000);
        current_item->set_callbacks();
        current_item->print();
      }
    }

    void back() {
      current_item = findParent(this);
      print();
    }

    void scroll_right() {
      scroll(1);
    }

    void scroll_left()  {
      scroll(-1);
    }

    void scroll_up()    {
      scroll(10);
    }

    void scroll_down()  {
      scroll(-10);
    }

    Folder * findParent(Folder *current_folder) {
      if (!current_folder) {
        return NULL;
      }

      for (MenuItem *child : current_folder->children) {
        if (this == child) {
          return current_folder;
        }
        if (child->item_type == kFolder) {
          return findParent((Folder*)child);
        }
      }
    }
};

Folder root_folder("", {
  new Folder("Match", {
    new Folder("One", {}),
    new Folder("Two", {})
  }),
  new Folder("Skills", {
  }),
  new Folder("Auto Waypoint", {
  }),
  new Folder("Other", {
  }),
  new Folder("Actions", {
  }),
  new Folder("Settings", {
  })
});

// void folder::scroll(int direction) {
  // switch (database[current_item].item_type) {
  //   int tempCursorLocation;
  //   int tempSettingValue;
  // case kFolder:
  //   tempCursorLocation = database[current_item].cursor_location + direction;
  //   if (tempCursorLocation < 0) {
  //     tempCursorLocation = database[current_item].items.size() - 1;
  //   } else if (tempCursorLocation > database[current_item].items.size() - 1) {
  //     tempCursorLocation = 0;
  //   }

  //   database[current_item].cursor_location = tempCursorLocation;
  //   break;
  // case kSettingSlider:
  //   tempSettingValue = database[current_item].setting_value + direction;
  //   if (database[current_item].setting_value + direction < 0) {
  //     tempSettingValue = 0;
  //   } else if (database[current_item].setting_value + direction > 100) {
  //     tempSettingValue = 100;
  //   }
  //   database[current_item].setting_value = tempSettingValue;
  //   break;
  // default:
  //   break;
  // }
  // print_menu();
// }
























// void print_auton() {
//   master.clear();
//   pros::delay(50);
//   master.print(0, 0, database[current_item].name);
// }

// void print_setting_slider() {
//   std::string bar = "l.................l";
//   int barValue = database[current_item].setting_value * 0.17;
//   bar.replace(1, barValue, barValue, '!');

//   master.clear();
//   pros::delay(50);
//   master.print(0, 0, database[current_item].name);
//   master.print(1, 0, bar.c_str());
//   master.print(2, 0, "%d%%", database[current_item].setting_value);
// }

// void print_setting_switch() {
//   master.clear();
//   pros::delay(50);
//   master.print(0, 0, database[current_item].name);
//   if (database[current_item].setting_value) {
//     master.print(1, 0, "True");
//   } else {
//     master.print(1, 0, "False");
//   }
// }

// void print_action(const char *message) {
//   master.clear();
//   pros::delay(50);
//   master.print(0, 0, database[current_item].name);
//   master.print(1, 0, database[current_item].description);
//   master.print(2, 0, message);
// }





// void scroll_right() {
//   current_item->scroll_right();
// }

// void scroll_left() {
//   current_item->scroll_left();
// }

// void scroll_up() {
//   current_item->scroll_up();
// }

// void scroll_down() {
//   current_item->scroll_down();
// }

// void back() {
//   current_item->back(&current_item);
// }

// void select() {
//   current_item->select(&current_item);
// }

void print_menu(MenuItem * item) {
  item->print();
}

// void run_auton() {
//   if (database[current_item].item_type == kAutonomous) {
//     master.rumble(".");
//     database[current_item].function();
//   }
// }

// void check_for_auton() {
//   if (database[current_item].item_type != kAutonomous) {
//     master.rumble(".");
//   }
// }



void set_callbacks() {
//   using namespace controllerbuttons;
//   button_callbacks = {
//     {&master, BTN_RIGHT, false, {&menu}, &scroll_right},
//     {&master, BTN_LEFT,  false, {&menu}, &scroll_left},
//     {&master, BTN_UP,    false, {&menu}, &scroll_up},
//     {&master, BTN_DOWN,  false, {&menu}, &scroll_down},
//     {&master, BTN_A,     false, {&menu}, &select},
//     {&master, BTN_B,     false, {&menu}, &back},
//   };
}

void init() {
  master.clear();
  pros::delay(50);
  current_item = &root_folder;
  pros::Task controller_print_task (controller_print);
  current_item->set_callbacks();
  print_menu(current_item);
}

} // namespace controllermenu