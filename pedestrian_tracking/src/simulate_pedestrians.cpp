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
      declare_parameter("rate", 2.0);
      declare_parameter("n_peds", 1);

      rate_hz = get_parameter("rate").as_double();
      std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / rate_hz));

      n_peds = get_parameter("n_peds").as_int();

      pedestrian_pub_ = create_publisher<crowd_nav_interfaces::msg::PedestrianArray>("pedestrians", 10);

      timer_ = create_wall_timer(rate, std::bind(&SimulatePedestrians::timer_callback, this));
    }

  private:
    double rate_hz;
    int n_peds;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<crowd_nav_interfaces::msg::PedestrianArray>::SharedPtr pedestrian_pub_;

    void timer_callback()
    {
      crowd_nav_interfaces::msg::PedestrianArray peds;
      const auto current_time = this->get_clock()->now();

      int max_peds = 3;
      std::vector<double> pedx{1.0,  2.0,  2.5};
      std::vector<double> pedy{0.0, -0.5,  0.5};

      for (int ped_counter = 0; ((ped_counter < n_peds) && (ped_counter < max_peds)); ped_counter++){
        crowd_nav_interfaces::msg::Pedestrian pedi;
        pedi.header.stamp = current_time;
        pedi.id = ped_counter + 1;
        pedi.pose.position.x = pedx.at(ped_counter);
        pedi.pose.position.y = pedy.at(ped_counter);
        peds.pedestrians.push_back(pedi);
      }

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