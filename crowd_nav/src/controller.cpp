#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "image_geometry/stereo_camera_model.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "crowd_nav_interfaces/msg/pixel.hpp"
#include "crowd_nav_interfaces/msg/pixel_array.hpp"
#include <opencv2/highgui.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class PathPlan : public rclcpp::Node
{
  public:
    Controller()
    : Node("controller")
    {

      auto pd = rcl_interfaces::msg::ParameterDescriptor{};

      pd.description = "Timer rate (Hz)";
      declare_parameter("rate", 2., pd);

      rate_hz = get_parameter("rate").as_double();
      RCLCPP_INFO_STREAM(get_logger(), "Rate is " << ((int)(1000. / rate_hz)) << "ms");
      std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / rate_hz));

      people_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "pedestrians", 10, std::bind(&PathPlan::pedestrians_cb, this, std::placeholders::_1));

      cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

      timer_ = create_wall_timer(
      rate, std::bind(&PathPlan::timer_callback, this));

    }

  private:
    double rate_hz;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr people_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    void pedestrians_cb(const visualization_msgs::msg::MarkerArray & msg)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Received pedestrians");
    }

    void timer_callback()
    {
      RCLCPP_INFO_STREAM(get_logger(), "Timer tick");
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlan>());
  rclcpp::shutdown();
  return 0;
}