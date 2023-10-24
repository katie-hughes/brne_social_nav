#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "crowd_nav_interfaces/msg/pedestrian.hpp"
#include "crowd_nav_interfaces/msg/pedestrian_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "zed_interfaces/msg/objects_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"


using namespace std::chrono_literals;

class ConvertPeds : public rclcpp::Node
{
  public:
    ConvertPeds()
    : Node("convert_peds")
    {
      zed_sub_ = create_subscription<zed_interfaces::msg::ObjectsStamped>(
        "zed2i/zed_node/obj_det/objects", 10, std::bind(&ConvertPeds::zed_cb, this, std::placeholders::_1));
      pedestrian_pub_ = create_publisher<crowd_nav_interfaces::msg::PedestrianArray>("pedestrians", 10);

      // From: https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/
      // Writing-A-Tf2-Broadcaster-Cpp.html
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      tf_buffer_= std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_= std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

  private:
    double rate_hz;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr zed_sub_;
    rclcpp::Publisher<crowd_nav_interfaces::msg::PedestrianArray>::SharedPtr pedestrian_pub_;

    void zed_cb(const zed_interfaces::msg::ObjectsStamped & msg)
    {
      crowd_nav_interfaces::msg::PedestrianArray pa;
      pa.header.stamp = msg.header.stamp;
      const int n_peds = msg.objects.size();
      // RCLCPP_INFO_STREAM(get_logger(), "\n\nMESSAGE");
      for (int i = 0; i < n_peds; i++){
        const auto p = msg.objects.at(i);
        // These positions are in zed frame. Need to be converted into odom frame.
        // frame id is always zed2i_left_camera_frame
        const std::string ped_name = "ped_"+std::to_string(p.label_id);
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = msg.header.stamp;
        t.header.frame_id = "zed2i_left_camera_frame";
        t.child_frame_id = ped_name;
        t.transform.translation.x = p.position.at(0);
        t.transform.translation.y = p.position.at(1);
        t.transform.translation.z = p.position.at(2);
        // Send the transformation
        tf_broadcaster_->sendTransform(t);
        // immediately read this tf to get the transform. Seems a little janky but works for now.
        geometry_msgs::msg::TransformStamped T_odom_ped;
        try {
          T_odom_ped = tf_buffer_->lookupTransform(
            "odom", ped_name,
            tf2::TimePointZero);
          crowd_nav_interfaces::msg::Pedestrian ped;
          ped.id = p.label_id;
          ped.pose.position.x = T_odom_ped.transform.translation.x;
          ped.pose.position.y = T_odom_ped.transform.translation.y;
          ped.pose.position.z = T_odom_ped.transform.translation.z;
          pa.pedestrians.push_back(ped);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO_STREAM(get_logger(), "Could not transform");
        }
      }
      pedestrian_pub_->publish(pa);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConvertPeds>());
  rclcpp::shutdown();
  return 0;
}