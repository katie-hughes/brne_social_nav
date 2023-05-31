#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "image_geometry/stereo_camera_model.h"
#include "sensor_msgs/msg/camera_info.hpp"

using namespace std::chrono_literals;

class TrackPedestrians : public rclcpp::Node
{
  public:
    TrackPedestrians()
    : Node("track_pedestrians"), count_(0)
    {

      auto pd = rcl_interfaces::msg::ParameterDescriptor{};

      pd.description = "Timer rate (Hz)";
      declare_parameter("rate", 200., pd);

      rate_hz = get_parameter("rate").as_double();
      RCLCPP_INFO_STREAM(get_logger(), "Rate is " << ((int)(1000. / rate_hz)) << "ms");
      std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / rate_hz));
      publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);
      // pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      //   "rslidar_points", 10, std::bind(&TrackPedestrians::pc_cb, this, std::placeholders::_1));
      timer_ = create_wall_timer(
      rate, std::bind(&TrackPedestrians::timer_callback, this));
    }

  private:
    double rate_hz;
    rclcpp::TimerBase::SharedPtr timer_;
    image_geometry::StereoCameraModel stereo;
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
  rclcpp::spin(std::make_shared<TrackPedestrians>());
  rclcpp::shutdown();
  return 0;
}