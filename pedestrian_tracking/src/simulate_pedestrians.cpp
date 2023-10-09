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

      pedestrian_pub_ = create_publisher<crowd_nav_interfaces::msg::PedestrianArray>("pedestrians", 10);

      timer_ = create_wall_timer(
      rate, std::bind(&SimulatePedestrians::timer_callback, this));
    }

  private:
    double rate_hz;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<crowd_nav_interfaces::msg::PedestrianArray>::SharedPtr pedestrian_pub_;

    void timer_callback()
    {
      crowd_nav_interfaces::msg::PedestrianArray peds;
      const auto current_time = this->get_clock()->now();
      // create a fake pedestrian with an id of 1
      crowd_nav_interfaces::msg::Pedestrian ped1;
      ped1.header.stamp = current_time;
      ped1.id = 1;
      ped1.pose.position.y = 1.0;
      peds.pedestrians.push_back(ped1);
      pedestrian_pub_->publish(peds);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimulatePedestrians>());
  rclcpp::shutdown();
  return 0;
}