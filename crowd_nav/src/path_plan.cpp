#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <armadillo>

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
      // define parameters
      declare_parameter("replan_freq", 1.0);
      declare_parameter("dt", 0.1);
      declare_parameter("maximum_agents", 1);
      declare_parameter("n_samples", 1);
      declare_parameter("n_steps", 1);
      declare_parameter("cost_a1", 1.0);
      declare_parameter("cost_a2", 1.0);
      declare_parameter("cost_a3", 1.0);
      declare_parameter("kernel_a1", 1.0);
      declare_parameter("kernel_a2", 1.0);
      declare_parameter("y_min", -1.0);
      declare_parameter("y_max", 1.0);
      declare_parameter("max_ang_vel", 1.0);

      // get parameters
      replan_freq = get_parameter("replan_freq").as_double();
      dt = get_parameter("dt").as_double();
      maximum_agents = get_parameter("maximum_agents").as_int();
      n_samples = get_parameter("n_samples").as_int();
      n_steps = get_parameter("n_steps").as_int();
      cost_a1 = get_parameter("cost_a1").as_double();
      cost_a2 = get_parameter("cost_a2").as_double();
      cost_a3 = get_parameter("cost_a3").as_double();
      kernel_a1 = get_parameter("kernel_a1").as_double();
      kernel_a2 = get_parameter("kernel_a2").as_double();
      y_min = get_parameter("y_min").as_double();
      y_max = get_parameter("y_max").as_double();

      // print out parameters
      RCLCPP_INFO_STREAM(get_logger(), "Replan frequency: " << replan_freq << " Hz");
      RCLCPP_INFO_STREAM(get_logger(), "dt: " << dt);
      RCLCPP_INFO_STREAM(get_logger(), "Maximum agents: " << maximum_agents);
      RCLCPP_INFO_STREAM(get_logger(), "Number of samples: " << n_samples);
      RCLCPP_INFO_STREAM(get_logger(), "Number of timesteps: " << n_steps);
      RCLCPP_INFO_STREAM(get_logger(), "Costs: " << cost_a1 << " " << cost_a2 << " " << cost_a3);
      RCLCPP_INFO_STREAM(get_logger(), "Kernels: " << kernel_a1 << " " << kernel_a2);
      RCLCPP_INFO_STREAM(get_logger(), "Hallway: " << y_min << "->" << y_max);

      brne = brne::BRNE{kernel_a1, kernel_a2,
                        cost_a1, cost_a2, cost_a3,
                        dt, n_steps, n_samples,
                        y_min, y_max};

      // Print out the parameters of the BRNE object to make sure it got initialized right
      RCLCPP_INFO_STREAM(get_logger(), brne.param_string());

      // Define publishers and subscribers
      pedestrian_sub_ = create_subscription<crowd_nav_interfaces::msg::PedestrianArray>(
        "pedestrians", 10, std::bind(&PathPlan::pedestrians_cb, this, std::placeholders::_1));
      cmd_buf_pub_ = create_publisher<crowd_nav_interfaces::msg::TwistArray>("cmd_buf", 10);

      // Create a timer that executes at replan_freq
      std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / replan_freq));
      timer_ = create_wall_timer(rate, std::bind(&PathPlan::timer_callback, this));

    }

  private:
    double replan_freq, kernel_a1, kernel_a2, cost_a1, cost_a2, cost_a3, y_min, y_max, dt, 
           max_ang_vel, max_lin_vel;
    int maximum_agents, n_samples, n_steps;

    int n_peds = 0;
    int n_prev_peds = 0;

    brne::BRNE brne{};

    rclcpp::TimerBase::SharedPtr timer_;

    arma::mat prev_ped_array;
    arma::mat ped_array;
    // crowd_nav_interfaces::msg::PedestrianArray ped_buffer;

    rclcpp::Subscription<crowd_nav_interfaces::msg::PedestrianArray>::SharedPtr pedestrian_sub_;

    rclcpp::Publisher<crowd_nav_interfaces::msg::TwistArray>::SharedPtr cmd_buf_pub_;

    void pedestrians_cb(const crowd_nav_interfaces::msg::PedestrianArray & msg)
    {
      // save values from previous iteration
      prev_ped_array = ped_array;
      n_prev_peds = n_peds;
      // start reading in message data
      n_peds = msg.pedestrians.size();
      ped_array = arma::mat(n_peds, 2, arma::fill::zeros);
      // iterate through pedestrians
      arma::mat for_compare(n_prev_peds, 2, arma::fill::zeros);
      for (int p=0; p<n_peds; p++){
        auto ped = msg.pedestrians.at(p);
        RCLCPP_INFO_STREAM(get_logger(), "Ped: " << ped.pose.position.x << "," << ped.pose.position.y);
        ped_array.at(p,0) = ped.pose.position.x;
        ped_array.at(p,1) = ped.pose.position.y;
        // make a matrix that looks like
        // x y
        // x y
        // ... number of rows = number of pedestrians.
        arma::rowvec f2f_vel(2, arma::fill::zeros);
        if (n_prev_peds > 0){
          arma::vec xs(n_prev_peds, arma::fill::value(ped.pose.position.x));
          arma::vec ys(n_prev_peds, arma::fill::value(ped.pose.position.y));
          for_compare.col(0) = xs;
          for_compare.col(1) = ys;
          arma::mat difference = prev_ped_array - for_compare;
          auto norm = arma::vecnorm(difference, 2, 1);
          f2f_vel = ped_array.row(p) - prev_ped_array.row(norm.index_min());
          RCLCPP_INFO_STREAM(get_logger(), "f2f vel: " << f2f_vel);
        }
        // msg.pedestrians.at(p).velocity.linear.x = f2f_vel.at(0);
        // msg.pedestrians.at(p).velocity.linear.y = f2f_vel.at(0);
      }
      RCLCPP_INFO_STREAM(get_logger(), "ped array\n" << ped_array);
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