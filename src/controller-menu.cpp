#include "api.h"

#include <bits/stdc++.h>
#include "json.hpp"
using json = nlohmann::ordered_json;

#include "robot-config.h"
#include "controller-buttons.h"
#include "controller-menu.h"
#include "robot-functions.h"
#include "auton-from-sd.h"
#include "auton-drive.h"

using namespace controllerbuttons;
namespace controllermenu {

std::string master_print_array [3];
std::string partner_print_array [3];

void controller_print() {
  std::string master_array_last [3];
  std::string partner_array_last [3];
  while (true) {
    for (int i = 0; i < 3; i++) {
      if (master_print_array[i] != master_array_last[i]) {
        master_array_last[i] = master_print_array[i];
        char print_str[20];
        master_print_array[i].resize(19);
        sprintf(print_str, "%-19s", master_print_array[i].c_str());
        master.print(i, 0, print_str);
        pros::delay(50);
      }
    }
    for (int i = 0; i < 3; i++) {
      if (partner_print_array[i] != partner_array_last[i]) {
        partner_array_last[i] = partner_print_array[i];
        char print_str[20];
        partner_print_array[i].resize(19);
        sprintf(print_str, "%-19s", partner_print_array[i].c_str());
        partner.print(i, 0, print_str);
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

  virtual void back();
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
    master_print_array[0] = selection;
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

  virtual void abort() {}

  void print() {
    abort();
    if (folderScroll.number_of_items > 0) {
      folderScroll.print();
      master_print_array[1] = MenuItemTypeName.at(children[folderScroll.cursor_location]->item_type);
      master_print_array[2] = children[folderScroll.cursor_location]->name;
    } else {
      master_print_array[0] = "Folder is empty";
      master_print_array[1] = "";
      master_print_array[2] = "";
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
  MenuCreateAuton(std::string name) : MenuItem(kAction, name), auton("1") {}

  void print() {
    chassis->setState({0_in, 0_in, 0_deg});
    pros::Task([this](){
      while (current_item == this) {
        double x = chassis->getState().x.convert(inch);
        double y = chassis->getState().y.convert(inch);
        double theta = chassis->getState().theta.convert(degree);
        std::string x_str     = std::to_string((int)x);
        std::string y_str     = std::to_string((int)y);
        std::string theta_str = std::to_string((int)theta);
        master_print_array[0] = "x: " + x_str + " y: " + y_str + " t: " + theta_str;
        master_print_array[1] = "step " + std::to_string(auton.selected_step + 1) + " of " + std::to_string(auton.auton_steps.size());
        // master_print_array[2] = "x: " + std::to_string((int)auton.auton_steps["x"])
        //     + " y: " + std::to_string((int)auton.auton_steps["y"])
        //     + " t: " + std::to_string((int)auton.auton_steps["theta"]);
        pros::delay(150);
      }
    });
  }

  void set_callbacks() {
    button_handler.clear_group("menu");
    button_handler.master.b.pressed.set ([this](){ back(); }, {"menu"});
    button_handler.master.a.pressed.set ([this](){
      auton.save();
      autonfromsd::save_autons_to_SD();
    }, {"menu"});
    button_handler.master.y.pressed.set ([this](){ auton.run(); }, {"menu"});
    button_handler.master.x.pressed.set ([this](){ auton.set_step_waypoint(); }, {"menu"});
    button_handler.master.up.pressed.set ([this](){ auton.insert_step(); }, {"menu"});
    button_handler.master.down.pressed.set ([this](){ auton.remove_step(); }, {"menu"});
    button_handler.master.left.pressed.set ([this](){ auton.previous_step(); }, {"menu"});
    button_handler.master.right.pressed.set ([this](){ auton.next_step(); }, {"menu"});
    button_handler.master.r1.pressed.set ([this](){ /*drive to nearest goal*/ }, {"menu"});
    button_handler.master.r2.pressed.set ([this](){ /*drive to and intake nearest ball*/ }, {"menu"});
    button_handler.master.l1.pressed.set ([this](){ /*score one ball if at goal*/ }, {"menu"});
    button_handler.master.l2.pressed.set ([this](){ /*intake one ball if at goal*/ }, {"menu"});
  }

  private:
  autonfromsd auton;
  // std::string auton_id = get_new_auton_id(autonfromsd::all_autons);
};

class MenuAutonomous : public MenuFolder {
  public:
  MenuAutonomous(std::string name, controllerbuttons::Macro routine) 
      : MenuFolder(name, {
          new MenuAction(this, "Select", "Selected:", "Waiting for enable", [this](){ select(); }),
          new MenuAction(this, "Test", "Running","Press 'B' to abort", [this](){ run(); })
        },
        kAutonomous), 
        routine_(routine) {}

  void select();

  void abort() {
    autondrive::auton_group.terminate();
  }
  
  void run() {
    routine_.start();
  }

  private:;
  controllerbuttons::Macro routine_;
};

extern MenuAutonomous &selected_auton;

void MenuAction::print() {
  master_print_array[0] = message_one_;
  master_print_array[1] = menu_autonomous_->name;
  master_print_array[2] = message_two_;
  function_();
}

void MenuAutonomous::select() {
  selected_auton = *this;
}
MenuAutonomous default_auton("None", autonroutines::none);

MenuAutonomous &selected_auton = default_auton;

void create_folder_structure() {
  root_folder = new MenuFolder("", {
    new MenuAutonomous("Home Row Three", autonroutines::home_row_three),
    new MenuAutonomous("Home Row Two", autonroutines::home_row_two),
    new MenuAutonomous("Left ShawnTon", autonroutines::left_shawnton),
    new MenuAutonomous("Right ShawnTon", autonroutines::right_shawnton),
    new MenuAutonomous("ShawnTon 3.0", autonroutines::shawnton_three),
    new MenuFolder("Other", {
      new MenuAutonomous("Skills", autonroutines::skills),
      new MenuAutonomous("None", autonroutines::none),
      new MenuAutonomous("Test", autonroutines::test),
    })
  });
}

void run_auton() {
  selected_auton.run();
}

void init() {
  master.clear();
  pros::delay(50);
  pros::Task controller_print_task (controller_print);
  create_folder_structure();
  current_item = root_folder;
  current_item->set_callbacks();
  current_item->print();
}

} // namespace controllermenu