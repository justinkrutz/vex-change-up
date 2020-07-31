#include "api.h"

#include <bits/stdc++.h>

#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"

namespace controllermenu {

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
    virtual void print(MenuItem * item) {}
    virtual void select(MenuItem ** item_ptr) {}
    virtual void back(MenuItem ** item_ptr) {}
    virtual void scroll_right() {}
    virtual void scroll_left() {}
    virtual void scroll_up() {}
    virtual void scroll_down() {}
    MenuItemType item_type;
    const char *name;
};


MenuItem *current_item;

class Folder : public MenuItem {
  private:
    int cursor_location = 0;
    void scroll(int direction) {
      int tempCursorLocation;
      int tempSettingValue;
      tempCursorLocation = this->cursor_location + direction;
      if (tempCursorLocation < 0) {
        tempCursorLocation = this->children.size() - 1;
      } else if (tempCursorLocation > this->children.size() - 1) {
        tempCursorLocation = 0;
      }
      this->cursor_location = tempCursorLocation;
    }

  public:
    Folder(const char *name_arg, std::vector<MenuItem> children_arg) {
      name = name_arg;
      children = children_arg;
      item_type = kFolder;
    }
    // const char *name;
    std::vector<MenuItem> children;

    void foo() {
      printf("foo");
    }

    void set_callbacks() {
      using namespace controllerbuttons;
      // button_callbacks[0].function = std::bind(&Folder::foo, this);
    }

    void print(MenuItem * item) {
      set_callbacks();
      printf("print_folder\n");
      Folder * item_to_print = (Folder*)item;
      std::string selection = "[_][_][_][_][_][_]";
      selection.resize(item_to_print->children.size() * 3);
      printf("cursor_location: %d\n", item_to_print->cursor_location);
      selection.replace((item_to_print->cursor_location) * 3 + 1, 1, "o");

      controller_print_array[0] = selection;
      controller_print_array[1] = "Type goes here";
      controller_print_array[1] = MenuItemTypeName.at(item_to_print->children[item_to_print->cursor_location].item_type);
      controller_print_array[2] = item_to_print->children[item_to_print->cursor_location].name;
    }

    void select(MenuItem **item_ptr) {
      printf("select\n");
      if (this->children.size() > 0) {
        *item_ptr = &this->children[this->cursor_location];
        print(current_item);
      }
    }

    void back(MenuItem ** item_ptr) {
      *item_ptr = findParent(this);
      print(current_item);
    }

    void scrollRight() { scroll(1); }

    void scrollLeft()  { scroll(-1); }

    void scrollUp()    { scroll(10); }

    void scrollDown()  { scroll(-10); }

    Folder * findParent(Folder *current_folder) {
      if (!current_folder) {
        return NULL;
      }

      for (MenuItem &child : current_folder->children) {
        if (this == &child) {
          return current_folder;
        }
        if (child.item_type == kFolder) {
          return findParent((Folder*)&child);
        }
      }
    }
};

Folder root_folder("", {
  Folder("Match", {
    Folder("One", {}),
    Folder("Two", {})
  }),
  Folder("Skills", {
  }),
  Folder("Auto Waypoint", {
  }),
  Folder("Other", {
  }),
  Folder("Actions", {
  }),
  Folder("Settings", {
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





void scroll_right() {
  current_item->scroll_right();
}

void scroll_left() {
  current_item->scroll_left();
}

void scroll_up() {
  current_item->scroll_up();
}

void scroll_down() {
  current_item->scroll_down();
}

void back() {
  current_item->back(&current_item);
}

void select() {
  current_item->select(&current_item);
}

void print_menu(MenuItem * item) {
  item->print(current_item);
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

controllerbuttons::MacroGroup menu;

void set_callbacks() {
  using namespace controllerbuttons;
  button_callbacks = {
    {&master, BTN_RIGHT, false, {&menu}, &scroll_right},
    {&master, BTN_LEFT,  false, {&menu}, &scroll_left},
    {&master, BTN_UP,    false, {&menu}, &scroll_up},
    {&master, BTN_DOWN,  false, {&menu}, &scroll_down},
    {&master, BTN_A,     false, {&menu}, &select},
    {&master, BTN_B,     false, {&menu}, &back},
  };
}

void init() {
  master.clear();
  pros::delay(50);
  current_item = &root_folder;
  pros::Task controller_print_task (controller_print);
  print_menu(current_item);
}

} // namespace controllermenu