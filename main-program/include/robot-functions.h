#ifndef ROBOT_FUNCTIONS_H
#define ROBOT_FUNCTIONS_H

#include "api.h"
#include <bits/stdc++.h>

// #define WARN(device, messageToPrint, warnIf)                                   \
//   if (warnIf) {                                                                \
//     const char *deviceChar = "" #device;                                       \
//     Controller1.Screen.clearScreen();                                          \
//     Controller1.Screen.setCursor(1, 0);                                        \
//     Controller1.Screen.print(deviceChar);                                      \
//     Controller1.Screen.setCursor(2, 0);                                        \
//     Controller1.Screen.print(messageToPrint);                                  \
//     wait(1, sec);                                                              \
//   }
//
// #define WARN_BOOL(device, member)                                              \
//   {                                                                            \
//     const char *message = "Not " #member;                                      \
//     WARN(device, message, !device.member)                                      \
//   }
//
// #define WARN_RANGE(device, member, lowerRange, upperRange)                     \
//   WARN(device, "Out of Range",                                                 \
//        !(device.member > lowerRange && device.member < upperRange))

namespace robotfunctions {
void count_up_task();
void count_down_task();
void count_task(void * arg);
void single_use_button();

void check_for_warnings();

void set_callbacks();
void motorTask();
} // namespace robotfunctions

#endif // ROBOT_FUNCTIONS_H
