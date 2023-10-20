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
#include <opencv2/highgui.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "zed_interfaces/msg/objects_stamped.hpp"

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
    }

  private:
    double rate_hz;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr zed_sub_;
    rclcpp::Publisher<crowd_nav_interfaces::msg::PedestrianArray>::SharedPtr pedestrian_pub_;

    void zed_cb(const zed_interfaces::msg::ObjectsStamped & msg)
    {
      crowd_nav_interfaces::msg::PedestrianArray pa;
      const int n_peds = msg.objects.size();
      RCLCPP_INFO_STREAM(get_logger(), "\n\nMESSAGE");
      for (int i = 0; i < n_peds; i++){
        const auto p = msg.objects.at(i);
        RCLCPP_INFO_STREAM(get_logger(), "id "<<p.label_id);
        RCLCPP_INFO_STREAM(get_logger(), "pos:"   <<p.position.at(0)<<" "<<p.position.at(1)<<" "<<p.position.at(2));
      }
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConvertPeds>());
  rclcpp::shutdown();
  return 0;
}