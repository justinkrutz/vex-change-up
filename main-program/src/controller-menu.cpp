#include "api.h"

#include <bits/stdc++.h>
#include <iostream>
#include <fstream>

#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"

using namespace controllerbuttons;

namespace controllermenu {

// controllerbuttons::MacroGroup menu;

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

class MenuItem;
MenuItem *current_item;

class Folder;
Folder *root_folder;


enum MenuItemType {kFolder,
                   kAutonomous,
                   kSettingSwitch,
                   kSettingSlider,
                   kAction};

std::map<MenuItemType, const char*> MenuItemTypeName = {
  {kFolder,        "Folder"},
  {kAutonomous,    "Autonomous"},
  {kSettingSwitch, "Setting Switch"},
  {kSettingSlider, "Setting Slider"},
  {kAction,        "Action"},
};

class MenuItem {
  public:
  virtual void print() {
    printf("virtual void print()\n");
  }

  virtual void set_callbacks() {
    printf("virtual void set_callbacks()\n");
  }

  Folder * findParent(Folder *current_folder = root_folder);

  void back();

    MenuItemType item_type;
    const char *name;
};

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

    button_handler.master.right.pressed.set ([this](){ scroll_right(); }, {"menu"});
    button_handler.master.left.pressed.set  ([this](){ scroll_left(); },  {"menu"});
    button_handler.master.up.pressed.set    ([this](){ scroll_up(); },    {"menu"});
    button_handler.master.down.pressed.set  ([this](){ scroll_down(); },  {"menu"});
    button_handler.master.a.pressed.set     ([this](){ select(); },       {"menu"});
    button_handler.master.b.pressed.set     ([this](){ back(); },         {"menu"});
  }

  void print() {
    // set_callbacks();
    printf("print_folder\n");
    // Folder * item_to_print = (Folder*)item;
    if (children.size() > 0) {
      std::string selection = "[_][_][_][_][_][_]";
      selection.resize(children.size() * 3);
      printf("cursor_location: %d\n", cursor_location);
      selection.replace((cursor_location) * 3 + 1, 1, "o");

      controller_print_array[0] = selection;
      controller_print_array[1] = MenuItemTypeName.at(children[cursor_location]->item_type);
      controller_print_array[2] = children[cursor_location]->name;
    } else {
      controller_print_array[0] = "Folder is empty";
      controller_print_array[1] = "";
      controller_print_array[2] = "";
    }
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
      // pros::delay(1000);
      current_item->set_callbacks();
      current_item->print();
    }
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
};

Folder * MenuItem::findParent(Folder *current_folder) {
  if (this == root_folder) {
    return root_folder;
  }
  for (MenuItem *child : current_folder->children) {
    if (this == child) {
      return current_folder;
    }
    if (child->item_type == kFolder) {
      Folder* sub_child = findParent((Folder*)child);
      if (sub_child) {
        return sub_child;
      }
    }
  }
  return NULL;
}

void MenuItem::back() {
  printf("back\n");
  printf("current_item before: %p\n", current_item);
  printf("current_item->name before: %s\n", current_item->name);
  current_item = findParent();
  printf("current_item after: %p\n", current_item);
  printf("current_item->name after: %s\n", current_item->name);
  // print();
  current_item->set_callbacks();
  current_item->print();
}

class Autonomous : public MenuItem {
  public:
  Autonomous(const char *name_arg) {
    name = name_arg;
    item_type = kAutonomous;
  }

  void print() {
  printf("print_autonomous\n");
    std::string selection = "[_][_][_]";
    printf("cursor_location: %d\n", cursor_location);
    selection.replace((cursor_location) * 3 + 1, 1, "o");

    controller_print_array[0] = name;
    controller_print_array[1] = selection;
    controller_print_array[2] = SaveStateName.at(cursor_location);
  }
  
  void set_callbacks() {
    button_handler.master.b.pressed.set ([this](){ back(); }, {"menu"});
  }

  private:
  enum SaveState {kDiscardChanges, kSaveNew, kOverwrite};
  SaveState cursor_location = kDiscardChanges;

  std::map<SaveState, const char*> SaveStateName = {
    {kDiscardChanges, "Discard Changes"},
    {kSaveNew,        "Save New"},
    {kOverwrite,      "Overwrite"},
  };

    void scroll(int direction) {
    printf("scroll\n");
    int temp_cursor_location;
    int temp_setting_value;
    temp_cursor_location = cursor_location + direction;
    if (temp_cursor_location < 0) {
      // temp_cursor_location = children.size() - 1;
    } else if (temp_cursor_location > 2) {
      temp_cursor_location = 0;
    }
    // cursor_location = temp_cursor_location;
    print();
  }
};

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

void createFolderStructure() {
  root_folder = new Folder("", {
  new Folder("Match", {
    new Folder("One", {}),
    new Folder("Two", {}),
  }),
  new Folder("Skills", {
  }),
  new Folder("Auton Builders", {
    new Autonomous("Build Match"),
    new Autonomous("Build Skills"),
  }),
  new Folder("Other", {
  }),
  new Folder("Actions", {
  }),
  new Folder("Settings", {
  })
});
}

void init() {
  createFolderStructure();
  master.clear();
  pros::delay(50);
  current_item = root_folder;
  pros::Task controller_print_task (controller_print);
  current_item->set_callbacks();
  print_menu(current_item);
}

} // namespace controllermenu