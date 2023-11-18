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
      int ped_counter = 0;
      crowd_nav_interfaces::msg::PedestrianArray peds;
      const auto current_time = this->get_clock()->now();
      if (ped_counter < n_peds){
        // create a fake pedestrian with an id of 1
        crowd_nav_interfaces::msg::Pedestrian ped1;
        ped1.header.stamp = current_time;
        ped1.id = 1;
        ped1.pose.position.x = 1.0;
        ped1.pose.position.y = 0.0;
        peds.pedestrians.push_back(ped1);
        ped_counter++;
      }
      
      if (ped_counter < n_peds){
        // create a fake pedestrian with an id of 2
        crowd_nav_interfaces::msg::Pedestrian ped2;
        ped2.header.stamp = current_time;
        ped2.id = 2;
        ped2.pose.position.x = 2.0;
        ped2.pose.position.y = -0.5;
        peds.pedestrians.push_back(ped2);
        ped_counter++;
      }
      
      if (ped_counter < n_peds){
        // ped 3
        crowd_nav_interfaces::msg::Pedestrian ped3;
        ped3.header.stamp = current_time;
        ped3.id = 3;
        ped3.pose.position.x = 2.5;
        ped3.pose.position.y = 0.5;
        peds.pedestrians.push_back(ped3);
        ped_counter++;
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