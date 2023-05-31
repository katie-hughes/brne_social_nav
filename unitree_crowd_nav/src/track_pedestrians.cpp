#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "image_geometry/stereo_camera_model.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include "unitree_crowd_nav_interfaces/msg/pixel.hpp"
#include "unitree_crowd_nav_interfaces/msg/pixel_array.hpp"

using namespace std::chrono_literals;

class TrackPedestrians : public rclcpp::Node
{
  public:
    TrackPedestrians()
    : Node("track_pedestrians"), count_(0)
    {

      auto pd = rcl_interfaces::msg::ParameterDescriptor{};

      pd.description = "Timer rate (Hz)";
      declare_parameter("rate", 2., pd);

      rate_hz = get_parameter("rate").as_double();
      RCLCPP_INFO_STREAM(get_logger(), "Rate is " << ((int)(1000. / rate_hz)) << "ms");
      std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / rate_hz));
      publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);

      pixel_head_front_left_sub_ = create_subscription<unitree_crowd_nav_interfaces::msg::PixelArray>(
        "/head/front/pixels_left", 10, std::bind(&TrackPedestrians::pixel_hfl_cb, this, std::placeholders::_1));
      pixel_head_front_right_sub_ = create_subscription<unitree_crowd_nav_interfaces::msg::PixelArray>(
        "/head/front/pixels_right", 10, std::bind(&TrackPedestrians::pixel_hfr_cb, this, std::placeholders::_1));

      timer_ = create_wall_timer(
      rate, std::bind(&TrackPedestrians::timer_callback, this));
    }

  private:
    double rate_hz;
    rclcpp::TimerBase::SharedPtr timer_;
    image_geometry::StereoCameraModel stereo;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<unitree_crowd_nav_interfaces::msg::PixelArray>::SharedPtr pixel_head_front_left_sub_;
    rclcpp::Subscription<unitree_crowd_nav_interfaces::msg::PixelArray>::SharedPtr pixel_head_front_right_sub_;

    size_t count_;

    void pixel_hfl_cb(const unitree_crowd_nav_interfaces::msg::PixelArray & msg)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Received Left");
    }

    void pixel_hfr_cb(const unitree_crowd_nav_interfaces::msg::PixelArray & msg)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Received Right");
    }

    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
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