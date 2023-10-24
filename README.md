# unitree_crowd_nav
Author: Katie Hughes

This set of ROS 2 packages implements Bayes Rule Nash Equilibrium (BRNE) crowd navigation on a Unitree Go1 robot as well as pedestrian tracking using a ZED2i camera. These packages are designed to be run on an external computer that is connected to the Go1 via its wireless hotspot.

To set up the necessary packages:
```
mkdir -p ${ws}/src
cd ${ws}/src
git clone git@github.com:katie-hughes/unitree_crowd_nav.git
cd ..
vcs import --recursive < src/unitree_crowd_nav/crowdnav.repos
colcon build
```

## Connecting to the robot
The Unitree Go1 has an onboard WLAN hotspot that is bridged to the ethernet connections between the boards. When the robot is powered on, check on your compter to see if the `UnitreeWifi` network is visible. In the event that it doesn't come up after a few minutes, the robot will need to be rebooted.

Once you connect to the wifi, you will need to configure a static IP on your own computer that matches the subnet `192.168.123.xxx`, which is what the boards on the robot are also set up with. (I usually set my IP to `192.168.123.100`). Then, you should be able to directly ssh into and see ros topics from all boards inside the dog.

## What needs to be running onboard the Go1 Xavier?
### Processing `cmd_vel`
The result of the BRNE algorithm is a motion plan represented as a series of `cmd_vel` commands. In order to process these commands, the [`unitree_nav`](https://github.com/ngmor/unitree_nav) package should be installed and running onboard the Go1 Xavier. To process the `cmd_vel`, run:
```
ros2 launch unitree_nav control.launch.py use_rviz:=false
```
### Creating `odom` updates
The onboard Xavier is also responsible for providing odometry updates of the robot. If you need to generate a map of the area, run the following launchfile. You should only need to do this once provided the resulting map is good.
```
ros2 launch unitree_nav mapping.launch.py use_rviz:=false restart_map:=true localize_only:=false
```
While you are mapping, you can 
Then to localize yourself within this map, run the following launchfile to get `/odom` updates.
```
ros2 launch unitree_nav mapping.launch.py use_rviz:=false restart_map:=false localize_only:=true
```

## What needs to be running on the external computer?

The external camera is responsible for detecting pedestrians as well as running the BRNE algorithm. 

The core launchfile is below. You can see the various arguments with `--show-args`.
```
ros2 launch crowd_nav crowdnav.launch.xml
```


To do everything in simulation -- simulated odometry updates as well as simulated pedestrians -- run:
```
ros2 launch crowd_nav crowdnav_sim.launch.xml
```

For external ZED tracking but onboard odometry run:
```
ros2 launch crowd_nav crowdnav_external_zed.launch.xml
```

Finally, for the ZED tracking to be done onboard the robot, run:
```
ros2 launch crowd_nav crowdnav_onboard_zed.launch.xml
```


## ROS 2 Packages
- [brne_py](brne_py) - A python implementation of the BRNE algorithm
- [crowd_nav_interfaces](crowd_nav_interfaces) - Defines some custom message types relevant to crowd navigation
- [crowd_nav](crowd_nav) - Framework for a C++ implementation of the BRNE algorithm. Still in progress.
- [pedestrian_tracking](pedestrian_tracking) - Tracks, displays, and can simulate pedestrian locations.

## Libraries
- [brnelib](brnelib) - Framework for the BRNE algorithm in C++. Still in progress.