#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "image_geometry/stereo_camera_model.h"

using namespace std::chrono_literals;

class FilterPC : public rclcpp::Node
{
  public:
    FilterPC()
    : Node("filter_point_cloud"), count_(0)
    {
      publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
      // pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      //   "rslidar_points", 10, std::bind(&FilterPC::pc_cb, this, std::placeholders::_1));
      timer_ = create_wall_timer(
      500ms, std::bind(&FilterPC::timer_callback, this));
    }

  private:

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    size_t count_;

    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FilterPC>());
  rclcpp::shutdown();
  return 0;
}