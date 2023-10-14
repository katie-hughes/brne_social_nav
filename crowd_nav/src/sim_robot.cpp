#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class SimRobot : public rclcpp::Node
{
  public:
    SimRobot()
    : Node("sim_robot")
    {
      auto pd = rcl_interfaces::msg::ParameterDescriptor{};

      pd.description = "Timer rate (Hz)";
      declare_parameter("rate", 2., pd);

      rate_hz = get_parameter("rate").as_double();
      RCLCPP_INFO_STREAM(get_logger(), "Rate is " << ((int)(1000. / rate_hz)) << "ms");
      std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / rate_hz));

      cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&SimRobot::cmd_vel_cb, this, std::placeholders::_1));

      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

      timer_ = create_wall_timer(
      rate, std::bind(&SimRobot::timer_callback, this));

    }

  private:
    double rate_hz;
    nav_msgs::msg::Odometry odom;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    void cmd_vel_cb(const geometry_msgs::msg::Twist & msg)
    {
      const auto vx = msg.linear.x;
      const auto vy = msg.linear.y;
      const auto vw = msg.angular.z;
      RCLCPP_INFO_STREAM(get_logger(), "Cmd vel:"<<vx<<" "<<vy<<" "<<vw);
      odom_pub_->publish(odom);
    }

    void timer_callback()
    {      
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimRobot>());
  rclcpp::shutdown();
  return 0;
}