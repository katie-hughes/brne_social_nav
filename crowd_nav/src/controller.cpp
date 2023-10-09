#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "image_geometry/stereo_camera_model.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "crowd_nav_interfaces/msg/twist_array.hpp"
#include <opencv2/highgui.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class Controller : public rclcpp::Node
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

      cmd_buf_sub_ = create_subscription<crowd_nav_interfaces::msg::TwistArray>(
        "cmd_buf", 10, std::bind(&Controller::cmd_buf_cb, this, std::placeholders::_1));

      cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

      timer_ = create_wall_timer(
      rate, std::bind(&Controller::timer_callback, this));

    }

  private:
    double rate_hz;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<crowd_nav_interfaces::msg::TwistArray>::SharedPtr cmd_buf_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    void cmd_buf_cb(const crowd_nav_interfaces::msg::TwistArray & msg)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Received buffer. Size="<<msg.twists.size());
    }

    void timer_callback()
    {
      // RCLCPP_INFO_STREAM(get_logger(), "Timer tick");
      geometry_msgs::msg::Twist vel;
      cmd_vel_pub_->publish(vel);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Controller>());
  rclcpp::shutdown();
  return 0;
}