/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Justin Krutz                                              */
/*    Created:      Thursday Apr 30 2020                                      */
/*    Description:  Template for test projects                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

#include <cstdio>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>

#include <bits/stdc++.h>


int main() {
  vexcodeInit();

  while (1) {
    task::sleep(100);
  }
}
