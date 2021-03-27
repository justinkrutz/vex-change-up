# vex-change-up
GitHub repository for VEX Robotics team 3018E Paracord for the 2020-2021 season: Change Up.

## Features
* Drivetrain control
  * Slew controller prevents tipping and reduces motor wear
  * Scales speed of all wheels to maintain desired direction when motor command is over 100%

* Autonomous
  * Odometry position tracking
  * Position error correction when aligning with goals
  * Full use of holonomic drive
  * Set waypoints at coordinates, balls, or goals
  * Set offset distance and angle from waypoints
  * Drive to goal function that uses sensors to detect goal and aligns rotationally
  * Run in a separate thread using the Macro class

* Ball management system
  * Four line trackers sense balls in the robot
  * Automatically loads balls intaken or inserted manually into the front of the robot
  * Advances balls after scoring
  * Aware of balls inserted into the top of the robot
  * Confirms number of balls in the robot after every two seconds of inactivity
  * Intake specified number of balls
  * Score specified number of balls
  * Eject specified number of balls
  * Uses intakes to pop out the last ball when ejecting
  * Uses smart motor controller class to manage slew rate and prioritized commands
  * Tracks ball colors (working but unused) 
  * Queue up balls to score when sensors detect a goal (working but unused) 
  * Distance sensors detect balls and automatically intake when a button is held (working but unused) 

* Macro class
  * Macros run in separate threads and can have blocking code
  * Safely terminate using an exception caught by a custom wait function
  * Sub macros are simultaneously terminated
  * Clean up function that runs even if terminated
  * Macro groups prevent conflicting macros from running simultaneously

* Button handler
  * Assign functions and macros to pressed and released events
  * Buttons can be reassigned
  * Button groups enable clearing catagories of button assignments

* Controller Menu
  * Autonomous selection
  * Autonomous testing
  * Supports recursive folders
  * Side scrolling if more items are in a folder than can fit on screen