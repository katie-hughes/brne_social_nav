# brne_py
Authors: Muchen Sun, Katie Hughes

This is a ROS 2 package containing a python implementation of Bayes Rule Nash Equilibrium.

## ROS Nodes
* `brne_nav`: Provides controls for the robot given odometry and pedestrian information.
* `brne_nav_class`: attempt at a class based BRNE algorithm (not working)

## Python libraries
* `brne.py`: Defines main BRNE functions
* `traj_tracker.py`: Gets controls to follow a trajectory
* `brne_class.py`: attempt at class based BRNE algorithm. Having trouble getting it to work with numba.

## Tests
* `test_brne.py`: working through testing what happens in one iteration of BRNE in a non ROS context.
* `test_brne.py`: working through testing what happens in one iteration of the BRNE class in a non ROS context.

