# unitree_crowd_nav
Author: Katie Hughes

This set of ROS 2 packages implements Bayes Rule Nash Equilibrium (BRNE) crowd navigation on a Unitree Go1 robot.

These packages are designed to run on an external computer. The result of this algorithm is a motion plan represented as a series of `cmd_vel` commands. In order to process these commands, the [`unitree_nav`](https://github.com/ngmor/unitree_nav) package should be installed and running onboard the Go1 (preferrably Xavier). To process the `cmd_vel`, run:
```
ros2 launch unitree_nav control.launch.py
```

Then, to set up the publishing of `cmd_vel` on your own computer, run:
```
ros2 launch crowd_nav py_crowdnav.launch.xml
```

## ROS 2 Packages
- [brne_py](brne_py) - A python implementation of the BRNE algorithm
- [crowd_nav_interfaces](crowd_nav_interfaces) - Defines some custom message types relevant to crowd navigation
- [crowd_nav](crowd_nav) - Framework for a C++ implementation of the BRNE algorithm. Still in progress.
- [pedestrian_tracking](pedestrian_tracking) - Framework for tracking and publishing pedestrian locations, either simulated or from a camera stream. Still in progress.

## Libraries
- [brnelib](brnelib) - Framework for the BRNE algorithm in C++. Still in progress.
