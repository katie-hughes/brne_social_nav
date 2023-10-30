# crowd_nav
Author: Katie Hughes

This is a ROS 2 package for setting up crowd navigation on a Unitree Go1 with the BRNE algorithm.


## ROS Nodes
* `sim_robot`: simulates odometry updates for a diff drive robot, useful for testing
* `pub_goal`: provides a service to manually set a specific goal pose
* `path_plan` (unfinished): framework to implement the C++ BRNE code.
* `controller` (unfinished): framework to execute the planned trajecctory.