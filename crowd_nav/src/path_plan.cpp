#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "image_geometry/stereo_camera_model.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include "crowd_nav_interfaces/msg/pedestrian_array.hpp"
#include "crowd_nav_interfaces/msg/twist_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "brnelib/brne.hpp"

using namespace std::chrono_literals;

class PathPlan : public rclcpp::Node
{
  public:
    PathPlan()
    : Node("plan_paths")
    {

      auto pd = rcl_interfaces::msg::ParameterDescriptor{};

      pd.description = "Timer rate (Hz)";
      declare_parameter("replan_freq", 1.0, pd);
      pd.description = "dt";
      declare_parameter("dt", 0.1, pd);
      pd.description = "Maximum agents BRNE will consider";
      declare_parameter("maximum_agents", 1, pd);
      pd.description = "Number of samples for BRNE optimization";
      declare_parameter("n_samples", 1, pd);
      pd.description = "Number of timesteps for BRNE optimization";
      declare_parameter("n_steps", 1, pd);
      pd.description = "cost a1";
      declare_parameter("cost_a1", 1.0, pd);
      pd.description = "cost a2";
      declare_parameter("cost_a2", 1.0, pd);
      pd.description = "cost a3";
      declare_parameter("cost_a3", 1.0, pd);
      pd.description = "kernel_a1";
      declare_parameter("kernel_a1", 1.0, pd);
      pd.description = "kernel_a2";
      declare_parameter("kernel_a2", 1.0, pd);
      pd.description = "minimum y coordinate of cooridor";
      declare_parameter("y_min", -1.0, pd);
      pd.description = "maximum y coordinate of cooridor";
      declare_parameter("y_max", 1.0, pd);
      pd.description = "maximum ang vel of robot";
      declare_parameter("max_ang_vel", 1.0, pd);


      replan_freq = get_parameter("replan_freq").as_double();
      RCLCPP_INFO_STREAM(get_logger(), "Replan frequency: " << replan_freq << " Hz");
      dt = get_parameter("dt").as_double();
      RCLCPP_INFO_STREAM(get_logger(), "dt: " << dt);
      maximum_agents = get_parameter("maximum_agents").as_int();
      RCLCPP_INFO_STREAM(get_logger(), "Maximum agents: " << maximum_agents);
      n_samples = get_parameter("n_samples").as_int();
      RCLCPP_INFO_STREAM(get_logger(), "Number of samples: " << n_samples);
      n_steps = get_parameter("n_steps").as_int();
      RCLCPP_INFO_STREAM(get_logger(), "Number of timesteps: " << n_steps);
      cost_a1 = get_parameter("cost_a1").as_double();
      cost_a2 = get_parameter("cost_a2").as_double();
      cost_a3 = get_parameter("cost_a3").as_double();
      RCLCPP_INFO_STREAM(get_logger(), "Costs: " << cost_a1 << " " << cost_a2 << " " << cost_a3);
      kernel_a1 = get_parameter("kernel_a1").as_double();
      kernel_a2 = get_parameter("kernel_a2").as_double();
      RCLCPP_INFO_STREAM(get_logger(), "Kernels: " << kernel_a1 << " " << kernel_a2);
      y_min = get_parameter("y_min").as_double();
      y_max = get_parameter("y_max").as_double();
      RCLCPP_INFO_STREAM(get_logger(), "Hallway: " << y_min << "->" << y_max);

      brne::BRNE brne_tmp{kernel_a1, kernel_a2,
                          cost_a1, cost_a2, cost_a3,
                          dt, n_steps, n_samples,
                          y_min, y_max};

      pedestrian_sub_ = create_subscription<crowd_nav_interfaces::msg::PedestrianArray>(
        "pedestrians", 10, std::bind(&PathPlan::pedestrians_cb, this, std::placeholders::_1));

      cmd_buf_pub_ = create_publisher<crowd_nav_interfaces::msg::TwistArray>("cmd_buf", 10);

      std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / replan_freq));
      timer_ = create_wall_timer(
      rate, std::bind(&PathPlan::timer_callback, this));

    }

  private:
    double replan_freq, kernel_a1, kernel_a2, cost_a1, cost_a2, cost_a3, y_min, y_max, dt, 
           max_ang_vel, max_lin_vel;
    int maximum_agents, n_samples, n_steps;

    brne::BRNE brne{};

    rclcpp::TimerBase::SharedPtr timer_;

    crowd_nav_interfaces::msg::PedestrianArray ped_buffer;

    rclcpp::Subscription<crowd_nav_interfaces::msg::PedestrianArray>::SharedPtr pedestrian_sub_;

    rclcpp::Publisher<crowd_nav_interfaces::msg::TwistArray>::SharedPtr cmd_buf_pub_;

    void pedestrians_cb(const crowd_nav_interfaces::msg::PedestrianArray & msg)
    {
      ped_buffer = msg;
      // RCLCPP_INFO_STREAM(get_logger(), "Received pedestrians, number="<<msg.pedestrians.size());
    }

    void timer_callback()
    {
      // RCLCPP_INFO_STREAM(get_logger(), "Timer tick");
      crowd_nav_interfaces::msg::TwistArray buf;
      cmd_buf_pub_->publish(buf);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlan>());
  rclcpp::shutdown();
  return 0;
}