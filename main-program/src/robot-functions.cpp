#include "api.h"

#include "robot-config.h"
#include "controller-buttons.h"
#include "robot-functions.h"


namespace robotfunctions {
// controllerbuttons::MacroGroup test1;
// controllerbuttons::MacroGroup test2;
// controllerbuttons::MacroGroup test3;
// controllerbuttons::MacroGroup abort;

// controllerbuttons::MacroGroup test_group;

// controllerbuttons::Macro count_up(
//     [](){
//       printf("start\n");
//       for (int i = 0; i < 50; i++) {
//         printf("Up %d\n", i);
//         controllerbuttons::wait(20);
//       }
//     }, 
//     [](){

//     },
//     {&test_group});

// Test function that prints to the terminal.
void single_use_button() {
  printf("single_use_button\n");
}

/*===========================================================================*/

void set_callbacks() {
  // using namespace controllerbuttons;
  // button_handler.master.a.pressed.set_macro(count_up);
  // button_handler.master.a.released.set([&](){ count_up.terminate(); });
  // button_handler.master.x.pressed.set([&](){ test_group.terminate(); });
  // button_handler.master.left.pressed.set(single_use_button, {}, {&test_group});
}

} // namespace robotfunctions
