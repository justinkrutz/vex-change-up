#include "api.h"

#include <bits/stdc++.h>
#include "json.hpp"
using json = nlohmann::ordered_json;

#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"
#include "autonomous.h"

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
    std::string name;
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
  Folder(std::string name_arg, std::vector<MenuItem*> children_arg) {
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
      printf("current_item->name: %s\n", current_item->name.c_str());
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
  printf("current_item->name before: %s\n", current_item->name.c_str());
  current_item = findParent();
  printf("current_item after: %p\n", current_item);
  printf("current_item->name after: %s\n", current_item->name.c_str());
  // print();
  current_item->set_callbacks();
  current_item->print();
}

class Autonomous : public MenuItem {
  public:
  Autonomous(std::string name_arg, std::string json_key_arg) {
    name = name_arg;
    json_key = json_key_arg;
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
  std::string json_key;

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


void print_menu(MenuItem * item) {
  item->print();
}

std::vector<controllermenu::MenuItem *> getAutonsFromJson(json autons, std::string auton_type) {
  std::vector<controllermenu::MenuItem *> auton_items;
  for (auto & auton : autons.items()) {
    // printf("autonType: %s\n", auton.value()["autonType"]);
    if (auton.value()["autonType"] == auton_type) {
      std::string auton_name;
      if (auton.value()["name"] == "") {
        auton_name = "Untitled-" + auton.key();
      } else {
        auton_name = auton.value()["name"];
      }
      auton_items.push_back(new Autonomous(auton_name, auton.key()));
    }
  }
  return auton_items;
}

void createFolderStructure() {
  root_folder = new Folder("", {
  new Folder("Match", {
    getAutonsFromJson(all_autons, "match")
  }),
  new Folder("Skills", {
    getAutonsFromJson(all_autons, "skills")
  }),
  new Folder("Auton Builders", {
    // new Autonomous("Build Match"),
    // new Autonomous("Build Skills"),
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