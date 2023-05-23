#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl_conversions/pcl_conversions.h>

using namespace std::chrono_literals;

class FilterPC : public rclcpp::Node
{
  public:
    FilterPC()
    : Node("filter_point_cloud"), count_(0)
    {
      publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
      pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "rslidar_points", 10, std::bind(&FilterPC::pc_cb, this, std::placeholders::_1));
      // timer_ = create_wall_timer(
      // 500ms, std::bind(&FilterPC::timer_callback, this));
    }

  private:

    // rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    size_t count_;

    // void timer_callback()
    // {
    //   auto message = std_msgs::msg::String();
    //   message.data = "Hello, world! " + std::to_string(count_++);
    //   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    //   publisher_->publish(message);
    // }
    void pc_cb(const sensor_msgs::msg::PointCloud2 & msg)
    {
      // const auto height = msg.height;
      // const auto width = msg.width;
      // const auto is_bigendian = msg.is_bigendian;
      // const auto point_step = msg.point_step;
      // const auto row_step = msg.row_step;
      // const auto data = msg.data;
      // RCLCPP_INFO_STREAM(get_logger(), "Width: "<<width<<" Height: "<<height);
      for (int i = 0; i < static_cast<int>(msg.fields.size()); i++) {
        const auto field = msg.fields.at(i);
        RCLCPP_INFO_STREAM(get_logger(), "Name: "<<field.name<<" Offset: "<<field.offset<<
                                         " datatype: "<<field.datatype<<" count: "<<field.count);
      }
      const auto npoints = msg.data.size();
      for (int i = 0; i < static_cast<int>(npoints); i++) {
        const int data = static_cast<int>(msg.data.at(i));
        RCLCPP_INFO_STREAM(get_logger(), "i: "<<i<<" data: "<<data);
      }
      RCLCPP_INFO_STREAM(get_logger(), "\n\n\n");
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FilterPC>());
  rclcpp::shutdown();
  return 0;
}