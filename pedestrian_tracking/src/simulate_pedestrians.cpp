#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "image_geometry/stereo_camera_model.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include "crowd_nav_interfaces/msg/pixel.hpp"
#include "crowd_nav_interfaces/msg/pixel_array.hpp"
#include <opencv2/highgui.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class SimulatePedestrians : public rclcpp::Node
{
  public:
    SimulatePedestrians()
    : Node("simulate_pedestrians")
    {
      auto pd = rcl_interfaces::msg::ParameterDescriptor{};

      pd.description = "Timer rate (Hz)";
      declare_parameter("rate", 2., pd);

      rate_hz = get_parameter("rate").as_double();
      RCLCPP_INFO_STREAM(get_logger(), "Rate is " << ((int)(1000. / rate_hz)) << "ms");
      std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / rate_hz));

      people_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("pedestrians", 10);

      timer_ = create_wall_timer(
      rate, std::bind(&SimulatePedestrians::timer_callback, this));
    }

  private:
    double rate_hz;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr people_pub_;

    void timer_callback()
    {
      RCLCPP_INFO_STREAM(get_logger(), "Track Ped");
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulatePedestrians>());
  rclcpp::shutdown();
  return 0;
}