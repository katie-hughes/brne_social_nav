# pedestrian_tracking
Author: Katie Hughes

This is a ROS 2 package designed to publish the location of pedestrians, either through the use of the ZED camera or simulated. 

## Nodes
* `convert_zed_ped`: This converts the ZED pedestrian message type (`zed_interfaces::msg::ObjectsStamped`) into my custom pedestrian message type (`crowd_nav_interfaces::msg::PedestrianArray`). It also converts the pedestrian locations into the `odom` frame by creating a `tf` frame for them.
* `show_pedestrians`: This node converts the `PedestrianArray` message type to markers are visualized as cylinders in RVIZ
* `simulate_pedestrians`: Publishes a simulated pedestrian at (1,0). (There is commmented out code that will publish more than one in other locations too. TODO: might make the number of pedestrians simulated a parameter).

## Launchfiles
* `zed.launch.xml`: Launches the ZED using the ZED ROS 2 wrapper with the appropriate configuration, converts to the `PedestrianArray` message type, and ensures continuity of `tf` frames with some static tf broadcasters.
* `perception.launch.xml`: Launches entire perception stack, either real or simulated. 