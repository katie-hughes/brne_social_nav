# brne_py
Authors: Muchen Sun, Katie Hughes

This is a ROS 2 package containing a python implementation of Bayes Rule Nash Equilibrium.

## ROS Nodes
* `brne_nav`: Provides controls for the robot given odometry and pedestrian information.

## Python libraries
* `brne.py`: Defines main BRNE functions
* `traj_tracker.py`: Gets controls to follow a trajectory

## Tests
* `test_brne.py`: working through testing what happens in one iteration of BRNE in a non ROS context.
* `test_cpp_sampling`: Takes samples generated from my C++ library of the BRNE functions, and feeds it through the rest of the python algorithm for verification that the final results are the same.
