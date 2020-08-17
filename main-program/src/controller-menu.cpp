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

std::string controller_print_array [3];

void controller_print() {
  std::string array_last [3];
  while (true) {
    for (int i = 0; i < 3; i++) {
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

class MenuFolder;
MenuFolder *root_folder;


enum MenuItemType {
  kFolder,
  kAutonomous,
  kAction,
};

std::map<MenuItemType, const char*> MenuItemTypeName = {
  {kFolder,           "Folder"},
  {kAutonomous,       "Autonomous"},
  {kAction, "Action"},
};


class MenuItem {
  public:
  MenuItem(MenuItemType item_type, std::string name, std::vector<MenuItem*> children = {})
           : item_type(item_type), name(name), children(children) {}

  MenuItemType item_type;
  std::string name;
  std::vector<MenuItem*> children;

  virtual void print() {
    printf("virtual void print()\n");
  }

  virtual void set_callbacks() {
    button_handler.clear_group("menu");
    button_handler.master.b.pressed.set ([this](){ back(); }, {"menu"});
  }

  MenuItem * findParent(MenuItem *current_folder = (MenuItem *) root_folder);

  void back();
};

class MenuScroll {
  public:
  MenuScroll(MenuItem *menu_item, int number_of_items) : menu_item(menu_item), number_of_items(number_of_items) {}

  int cursor_location = 0;

  int number_of_items;

  void print() {
    std::string selection = "[_][_][_][_][_]";
    if (number_of_items < 6) {
      selection.resize(number_of_items * 3);
      selection.replace(cursor_location * 3 + 1, 1, "o");
    } else if (cursor_location > 2 && cursor_location < number_of_items - 3) {
      selection.replace(1, 1, "<");
      selection.replace(13, 1, ">");
      selection.replace(7, 1, "o");
    } else if (cursor_location > 2) {
      selection.replace(1, 1, "<");
      selection.replace((cursor_location - number_of_items + 5) * 3 + 1, 1, "o");
    } else if (cursor_location < number_of_items - 3) {
      selection.replace(13, 1, ">");
      selection.replace(cursor_location * 3 + 1, 1, "o");
    }
    controller_print_array[0] = selection;
  }

  void set_callbacks() {
    button_handler.clear_group("menu");
    button_handler.master.right.pressed.set ([this](){ scroll(1); }, {"menu"});
    button_handler.master.left.pressed.set  ([this](){ scroll(-1); },  {"menu"});
  }

  private:
  MenuItem *menu_item;

  void scroll(int scroll_amount) {
    if (scroll_amount + cursor_location >= 0 && scroll_amount + cursor_location < number_of_items) {
      cursor_location += scroll_amount;
      menu_item->print();
    }
  }
};

class MenuFolder : public MenuItem {
  public:
  MenuFolder(std::string name, std::vector<MenuItem*> children, MenuItemType item_type = kFolder)
         : MenuItem(item_type, name, children), folderScroll(this, children.size()) {}

  void set_callbacks() {
    button_handler.clear_group("menu");
    folderScroll.set_callbacks();
    button_handler.master.a.pressed.set ([this](){ select(); }, {"menu"});
    button_handler.master.b.pressed.set ([this](){ back(); },   {"menu"});
  }

  void print() {
    if (folderScroll.number_of_items > 0) {
      folderScroll.print();
      controller_print_array[1] = MenuItemTypeName.at(children[folderScroll.cursor_location]->item_type);
      controller_print_array[2] = children[folderScroll.cursor_location]->name;
    } else {
      controller_print_array[0] = "Folder is empty";
      controller_print_array[1] = "";
      controller_print_array[2] = "";
    }
  }

  void select() {
    if (children.size() > 0) {
      current_item = children[folderScroll.cursor_location];
      current_item->set_callbacks();
      current_item->print();
    }
  }

  private:
  MenuScroll folderScroll;
};

MenuItem * MenuItem::findParent(MenuItem *current_folder) {
  if (this == root_folder) {
    return root_folder;
  }
  for (MenuItem *child : current_folder->children) {
    if (this == child) {
      return current_folder;
    }
    MenuItem* sub_child = findParent((MenuItem*)child);
    if (sub_child) {
      return sub_child;
    }
  }
  return NULL;
}

void MenuItem::back() {
  current_item = findParent();
  current_item->set_callbacks();
  current_item->print();
}

class MenuAutonomous;

class MenuAction : public MenuItem {
  public:
  MenuAction(MenuAutonomous *menu_autonomous,
             std::string name,
             std::string message_one,
             std::string message_two,
             std::function<void()> function)
           : MenuItem(kAction, name),
             message_one_(message_one),
             message_two_(message_two),
             menu_autonomous_(menu_autonomous),
             function_(function) {}

  void print();
  private:
  MenuAutonomous *menu_autonomous_;
  std::string message_one_;
  std::string message_two_;
  std::function<void()> function_;
};

class MenuCreateAuton : public MenuItem {
  public:
  MenuCreateAuton(std::string name) : MenuItem(kAction, name) {}

  void set_callbacks() {
    button_handler.clear_group("menu");
    button_handler.master.b.pressed.set ([this](){ back(); }, {"menu"});
    button_handler.master.a.pressed.set ([this](){ /*apply changes to waypoint*/ }, {"menu"});
    button_handler.master.x.pressed.set ([this](){ /*discard changes to waypoint*/ }, {"menu"});
    button_handler.master.up.pressed.set ([this](){ /*add waypoint*/ }, {"menu"});
    button_handler.master.down.pressed.set ([this](){ /*remove waypoint*/ }, {"menu"});
    button_handler.master.left.pressed.set ([this](){ /*got to previous waypoint*/ }, {"menu"});
    button_handler.master.right.pressed.set ([this](){ /*got to next waypoint*/ }, {"menu"});
    button_handler.master.r1.pressed.set ([this](){ /*drive to nearest goal*/ }, {"menu"});
    button_handler.master.r2.pressed.set ([this](){ /*drive to and intake nearest ball*/ }, {"menu"});
    button_handler.master.l1.pressed.set ([this](){ /*score one ball if at goal*/ }, {"menu"});
    button_handler.master.l2.pressed.set ([this](){ /*intake one ball if at goal*/ }, {"menu"});
  }

  private:
  // std::string auton_id = getNewAutonId(AutonManager::all_autons);
};

// class MenuAutonomousSelect : public MenuItem {
//   MenuAutonomousSelect(MenuAutonomous *menu_autonomous) : MenuItem(kAutonomousSelect, "Select for match"), menu_autonomous(menu_autonomous) {}

//   void print();
//   private:
//   MenuAutonomous *menu_autonomous;
// };

// class MenuAutonomousRun : public MenuItem {
//   public:
//   MenuAutonomousRun(MenuAutonomous *menu_autonomous) : MenuItem(kAutonomousRun, "Run immediately"), menu_autonomous(menu_autonomous) {}

//   void print();
//   private:
//   MenuAutonomous *menu_autonomous;
// };

// class MenuAutonomousEdit : public MenuItem {
//   public:
//   MenuAutonomousEdit(MenuAutonomous *menu_autonomous) : MenuItem(kAutonomousEdit, "Changes saved to SD"), menu_autonomous(menu_autonomous) {}

//   void print();
//   private:
//   MenuAutonomous *menu_autonomous;
// };



class MenuAutonomous : public MenuFolder {
  public:
  MenuAutonomous(std::string name, std::string json_key) 
      : MenuFolder(name, {
          new MenuAction(this, "Select", "Selected:", "Waiting for enable", [this](){ select(); }),
          new MenuAction(this, "Run", "Running","Press 'B' to abort", [this](){ run(); }),
          new MenuAction(this, "Edit", "Editing", "Press 'A' to save", [this](){ edit(); })
        },
        kAutonomous),
        json_key(json_key) {}

  void select() {}
  void run() {}
  void edit() {}

  private:
  std::string json_key;
};

void MenuAction::print() {
  controller_print_array[0] = message_one_;
  controller_print_array[1] = menu_autonomous_->name;
  controller_print_array[2] = message_two_;
  function_();
}







std::vector<MenuItem *> getMenuAutonsFromJson(json autons, std::string auton_type) {
  std::vector<MenuItem *> auton_items;
  for (auto & auton : autons.items()) {
    // printf("autonType: %s\n", auton.value()["autonType"]);
    if (auton.value()["autonType"] == auton_type) {
      std::string auton_name;
      if (auton.value()["name"] == "") {
        auton_name = "Untitled-" + auton.key();
      } else {
        auton_name = auton.value()["name"];
      }
      auton_items.push_back(new MenuAutonomous(auton_name, auton.key()));
    }
  }
  return auton_items;
}

void createFolderStructure() {
  root_folder = new MenuFolder("", {
  new MenuFolder("Match Autons", {
    getMenuAutonsFromJson(AutonManager::all_autons, "match")
  }),
  new MenuFolder("Skills Autons", {
    getMenuAutonsFromJson(AutonManager::all_autons, "skills")
  }),
  // new MenuFolder("Auton Builders", {
  //   // new Autonomous("Build Match"),
  //   // new Autonomous("Build Skills"),
  // }),
  // new MenuFolder("Other", {
  // }),
  // new MenuFolder("Actions", {
  // }),
  // new MenuFolder("Settings", {
  // })
});
}

void init() {
  createFolderStructure();
  master.clear();
  pros::delay(50);
  current_item = root_folder;
  pros::Task controller_print_task (controller_print);
  current_item->set_callbacks();
  current_item->print();
}

} // namespace controllermenu