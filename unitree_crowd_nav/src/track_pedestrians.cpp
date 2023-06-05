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

      pd.description = "Camera location on dog";
      declare_parameter("dog_cam_location", "head/front", pd);
      cam_loc = get_parameter("dog_cam_location").get_parameter_value().get<std::string>();

      RCLCPP_INFO_STREAM(get_logger(), "Using: " << cam_loc);

      publisher_ = create_publisher<std_msgs::msg::String>("topic", 10);

      const auto left_topic = '/' + cam_loc + "/pixels_left";
      const auto right_topic = '/' + cam_loc + "/pixels_right";

      const auto left_cam_info_topic = '/' + cam_loc + "/cam/image_rect/left/camera_info";
      const auto right_cam_info_topic = '/' + cam_loc + "/cam/image_rect/right/camera_info";

      RCLCPP_INFO_STREAM(get_logger(), "Left topic: " << left_topic);
      RCLCPP_INFO_STREAM(get_logger(), "Right topic: " << right_topic);

      pixel_left_sub_ = create_subscription<unitree_crowd_nav_interfaces::msg::PixelArray>(
        left_topic, 10, std::bind(&TrackPedestrians::pixel_left_cb, this, std::placeholders::_1));
      pixel_right_sub_ = create_subscription<unitree_crowd_nav_interfaces::msg::PixelArray>(
        right_topic, 10, std::bind(&TrackPedestrians::pixel_right_cb, this, std::placeholders::_1));

      info_left_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        left_cam_info_topic, 10, std::bind(&TrackPedestrians::info_left_cb, this, std::placeholders::_1));
      info_right_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        right_cam_info_topic, 10, std::bind(&TrackPedestrians::info_right_cb, this, std::placeholders::_1));

      timer_ = create_wall_timer(
      rate, std::bind(&TrackPedestrians::timer_callback, this));
    }

  private:
    double rate_hz;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    rclcpp::Subscription<unitree_crowd_nav_interfaces::msg::PixelArray>::SharedPtr pixel_left_sub_;
    rclcpp::Subscription<unitree_crowd_nav_interfaces::msg::PixelArray>::SharedPtr pixel_right_sub_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_right_sub_;

    size_t count_;

    std::string cam_loc;

    sensor_msgs::msg::CameraInfo info_left, info_right;

    bool info_left_received = false;
    bool info_right_received = false;

    unitree_crowd_nav_interfaces::msg::PixelArray left_pixels, right_pixels;

    image_geometry::StereoCameraModel stereo;

    void pixel_left_cb(const unitree_crowd_nav_interfaces::msg::PixelArray & msg)
    {
      left_pixels = msg;
      // for (int i = 0; i < static_cast<int>(msg.pixels.size()); i++){
      //   RCLCPP_INFO_STREAM(get_logger(), "Left center: ("<<msg.pixels.at(i).x << ","<<msg.pixels.at(i).y<<")");
      // }
    }

    void pixel_right_cb(const unitree_crowd_nav_interfaces::msg::PixelArray & msg)
    {
      right_pixels = msg;
      // for (int i = 0; i < static_cast<int>(msg.pixels.size()); i++){
      //   RCLCPP_INFO_STREAM(get_logger(), "Right center: ("<<msg.pixels.at(i).x << ","<<msg.pixels.at(i).y<<")");
      // }
    }

    void info_left_cb(const sensor_msgs::msg::CameraInfo & msg)
    {
      // RCLCPP_INFO_STREAM(get_logger(), "Received Left Info");
      info_left = msg;
      info_left_received = true;
    }

    void info_right_cb(const sensor_msgs::msg::CameraInfo & msg)
    {
      // RCLCPP_INFO_STREAM(get_logger(), "Received Right Info");
      info_right = msg;
      info_right_received = true;
    }

    void timer_callback()
    {
      // check if camera has been initialized
      if (!stereo.initialized() && (info_left_received && info_right_received)){
        RCLCPP_INFO_STREAM(get_logger(), "Initialize camera stupid");
        const auto success = stereo.fromCameraInfo(info_left, info_right);
        RCLCPP_INFO_STREAM(get_logger(), "Success???"<<success);
      }
      // in here I should analyze the pixels that come in and try to match them up.
      if (stereo.initialized()){
        for (int i = 0; i < static_cast<int>(left_pixels.pixels.size()); i++){
          RCLCPP_INFO_STREAM(get_logger(), "Left center: ("<<left_pixels.pixels.at(i).x << ","<<left_pixels.pixels.at(i).y<<")");
        }
        for (int i = 0; i < static_cast<int>(right_pixels.pixels.size()); i++){
          RCLCPP_INFO_STREAM(get_logger(), "Right center: ("<<right_pixels.pixels.at(i).x << ","<<right_pixels.pixels.at(i).y<<")");
        }
        // match them up 
        // compute disparity 
        // calculate 3d points
      }

      // auto message = std_msgs::msg::String();
      // message.data = "Hello, world! " + std::to_string(count_++);
      // // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      // publisher_->publish(message);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackPedestrians>());
  rclcpp::shutdown();
  return 0;
}