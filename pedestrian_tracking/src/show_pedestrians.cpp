#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "image_geometry/stereo_camera_model.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include "crowd_nav_interfaces/msg/pedestrian.hpp"
#include "crowd_nav_interfaces/msg/pedestrian_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class ShowPedestrians : public rclcpp::Node
{
public:
  ShowPedestrians()
  : Node("show_pedestrians")
  {

    pedestrian_sub_ = create_subscription<crowd_nav_interfaces::msg::PedestrianArray>(
      "pedestrians", 10, std::bind(&ShowPedestrians::pedestrians_cb, this, std::placeholders::_1));
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("ped_markers", 10);
  }

private:
  rclcpp::Subscription<crowd_nav_interfaces::msg::PedestrianArray>::SharedPtr pedestrian_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  visualization_msgs::msg::MarkerArray ma;

  void pedestrians_cb(const crowd_nav_interfaces::msg::PedestrianArray & msg)
  {
    // issue: if the pedestrian disappears, the marker will stay.
    // This is solved by making the lifetime small.
    const auto current_time = this->get_clock()->now();
    // clear previous marker array
    ma.markers.clear();
    const int n_pedestrians = msg.pedestrians.size();
    for (int i = 0; i < n_pedestrians; i++) {
      const auto ped = msg.pedestrians.at(i);
      visualization_msgs::msg::Marker m;
      m.header.stamp = current_time;
      m.header.frame_id = "brne_odom";
      m.id = i;   // ped.id;
      m.type = 3;         // cylinder
      m.action = 0;       // add/modify
      // Set color as orange
      m.color.r = 1.0;
      m.color.g = 0.67;
      m.color.b = 0.0;
      m.color.a = 1.0;
      // Set Radius
      m.scale.x = 0.1;
      m.scale.y = 0.1;
      m.scale.z = 0.25;
      m.pose.position.x = ped.pose.position.x;
      m.pose.position.y = ped.pose.position.y;
      m.pose.position.z = 0.125;
      m.lifetime.sec = 1;
      // Add to marker array
      ma.markers.push_back(m);
    }
    marker_pub_->publish(ma);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShowPedestrians>());
  rclcpp::shutdown();
  return 0;
}
