#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "crowd_nav_interfaces/msg/pedestrian_array.hpp"
#include "crowd_nav_interfaces/msg/twist_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "brnelib/brne.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

struct RobotPose{
  double x;
  double y;
  double theta;
  arma::rowvec toVec(){
    return arma::rowvec(std::vector<double>{x,y,theta});
  }
};

double dist(double x1, double y1, double x2, double y2){
  return sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

class PathPlan : public rclcpp::Node
{
  public:
    PathPlan()
    : Node("plan_paths"), goal_set{false}
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
      declare_parameter("people_timeout", 1.0);
      declare_parameter("goal_threshold", 1.0);
      declare_parameter("brne_activate_threshold", 1.0);
      declare_parameter("max_lin_vel", 1.0);
      declare_parameter("nominal_lin_vel", 1.0);
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
      people_timeout = get_parameter("people_timeout").as_double();
      goal_threshold = get_parameter("goal_threshold").as_double();
      brne_activate_threshold = get_parameter("brne_activate_threshold").as_double();
      max_ang_vel = get_parameter("max_ang_vel").as_double();
      max_lin_vel = get_parameter("max_lin_vel").as_double();
      nominal_lin_vel = get_parameter("nominal_lin_vel").as_double();

      // print out parameters
      RCLCPP_INFO_STREAM(get_logger(), "Replan frequency: " << replan_freq << " Hz");
      RCLCPP_INFO_STREAM(get_logger(), "dt: " << dt);
      RCLCPP_INFO_STREAM(get_logger(), "Maximum agents: " << maximum_agents);
      RCLCPP_INFO_STREAM(get_logger(), "Number of samples: " << n_samples);
      RCLCPP_INFO_STREAM(get_logger(), "Number of timesteps: " << n_steps);
      RCLCPP_INFO_STREAM(get_logger(), "Costs: " << cost_a1 << " " << cost_a2 << " " << cost_a3);
      RCLCPP_INFO_STREAM(get_logger(), "Kernels: " << kernel_a1 << " " << kernel_a2);
      RCLCPP_INFO_STREAM(get_logger(), "Hallway: " << y_min << "->" << y_max);
      RCLCPP_INFO_STREAM(get_logger(), "People timeout after " << people_timeout << "s");
      RCLCPP_INFO_STREAM(get_logger(), "Goal Threshold " << goal_threshold << "m");
      RCLCPP_INFO_STREAM(get_logger(), "Brne Activate Threshold " << brne_activate_threshold << "m");
      RCLCPP_INFO_STREAM(get_logger(), "Max Lin: " << max_lin_vel << " nominal lin: " << nominal_lin_vel << " max ang: " << max_ang_vel);

      brne = brne::BRNE{kernel_a1, kernel_a2,
                        cost_a1, cost_a2, cost_a3,
                        dt, n_steps, n_samples,
                        y_min, y_max};

      trajgen = brne::TrajGen{max_lin_vel, max_ang_vel, n_samples, n_steps, dt};

      // Print out the parameters of the BRNE object to make sure it got initialized right
      RCLCPP_INFO_STREAM(get_logger(), brne.param_string());

      // Define publishers and subscribers
      pedestrian_sub_ = create_subscription<crowd_nav_interfaces::msg::PedestrianArray>(
        "pedestrians", 10, std::bind(&PathPlan::pedestrians_cb, this, std::placeholders::_1));
      goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10, std::bind(&PathPlan::goal_cb, this, std::placeholders::_1));
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&PathPlan::odom_cb, this, std::placeholders::_1));
      cmd_buf_pub_ = create_publisher<crowd_nav_interfaces::msg::TwistArray>("cmd_buf", 10);
      path_pub_ = create_publisher<nav_msgs::msg::Path>("/optimal_path", 10);

      // Create a timer that executes at replan_freq
      std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / replan_freq));
      timer_ = create_wall_timer(rate, std::bind(&PathPlan::timer_callback, this));

    }

  private:
    double replan_freq, kernel_a1, kernel_a2, cost_a1, cost_a2, cost_a3, y_min, y_max, dt, 
           max_ang_vel, max_lin_vel, people_timeout, goal_threshold, brne_activate_threshold,
           nominal_lin_vel;
    int maximum_agents, n_samples, n_steps;

    int n_peds = 0;
    int n_prev_peds = 0;

    brne::BRNE brne{};
    brne::TrajGen trajgen{};

    rclcpp::TimerBase::SharedPtr timer_;

    arma::mat prev_ped_array;
    arma::mat ped_array;

    crowd_nav_interfaces::msg::PedestrianArray ped_buffer;
    crowd_nav_interfaces::msg::PedestrianArray selected_peds;

    crowd_nav_interfaces::msg::TwistArray robot_cmds;

    nav_msgs::msg::Path optimal_path;

    RobotPose robot_pose;

    rclcpp::Subscription<crowd_nav_interfaces::msg::PedestrianArray>::SharedPtr pedestrian_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<crowd_nav_interfaces::msg::TwistArray>::SharedPtr cmd_buf_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    bool goal_set;
    geometry_msgs::msg::PoseStamped goal;

    void goal_cb(const geometry_msgs::msg::PoseStamped & msg)
    {
      RCLCPP_INFO_STREAM(get_logger(), "Goal Received: " << msg.pose.position.x << ", " << msg.pose.position.y);
      goal_set = true;
      goal = msg;
      check_goal();
    }

    void odom_cb(const nav_msgs::msg::Odometry & msg)
    {
      // get the angle from the quaternion
      tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      robot_pose.x = msg.pose.pose.position.x;
      robot_pose.y = msg.pose.pose.position.y;
      robot_pose.theta = yaw;
      if (goal_set){
        check_goal();
      }
    }

    void check_goal(){
      auto dist_to_goal = dist(robot_pose.x, robot_pose.y, goal.pose.position.x, goal.pose.position.y);
      // RCLCPP_INFO_STREAM(get_logger(), "Dist to goal" << dist_to_goal);
      if (dist_to_goal < goal_threshold){
        RCLCPP_INFO_STREAM(get_logger(), "Goal Reached!");
        goal_set = false;
      }
    }

    void pedestrians_cb(const crowd_nav_interfaces::msg::PedestrianArray & msg)
    {
      // save values from previous iteration
      prev_ped_array = ped_array;
      n_prev_peds = n_peds;
      // make a copy of the message
      crowd_nav_interfaces::msg::PedestrianArray peds = msg;
      // start reading in message data
      n_peds = peds.pedestrians.size();
      ped_array = arma::mat(n_peds, 2, arma::fill::zeros);
      // iterate through pedestrians
      arma::mat for_compare(n_prev_peds, 2, arma::fill::zeros);
      for (int p=0; p<n_peds; p++){
        auto ped = peds.pedestrians.at(p);
        // RCLCPP_INFO_STREAM(get_logger(), "Ped: " << ped.pose.position.x << "," << ped.pose.position.y);
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
          // RCLCPP_INFO_STREAM(get_logger(), "f2f vel: " << f2f_vel);
        }
        peds.pedestrians.at(p).velocity.linear.x = f2f_vel.at(0);
        peds.pedestrians.at(p).velocity.linear.y = f2f_vel.at(0);
        // add to pedestrian buffer
        // first have to do some error checking to make sure the index is in bounds
        // initially length is 0 and I want to append pedestrian 0
        // while(0<1)go through
        // while(1<1)BREAK
        while (ped_buffer.pedestrians.size() < ped.id+1){
          crowd_nav_interfaces::msg::Pedestrian blank_ped;
          blank_ped.id = ped_buffer.pedestrians.size();
          ped_buffer.pedestrians.push_back(blank_ped);
        }
        ped_buffer.pedestrians.at(ped.id) = peds.pedestrians.at(p);
      }
      // RCLCPP_INFO_STREAM(get_logger(), "ped array\n" << ped_array);
    }

    void timer_callback()
    {
      robot_cmds.twists.clear();
      RCLCPP_INFO_STREAM(get_logger(), "\n\nTimer Tick");
      RCLCPP_INFO_STREAM(get_logger(), "Robot @: " << robot_pose.x << " " << robot_pose.y << " " << robot_pose.theta);
      // read in pedestrian buffer
      builtin_interfaces::msg::Time current_timestamp;
      current_timestamp = this->get_clock()->now();
      auto current_time = current_timestamp.sec + 1e-9 * current_timestamp.nanosec;
      // RCLCPP_INFO_STREAM(get_logger(), "Current time: " << current_timestamp.sec << "s " << current_timestamp.nanosec << "ns ");

      selected_peds.pedestrians.clear();
      std::vector<double> dists_to_peds;
      for(auto p:ped_buffer.pedestrians){
        auto ped_time = p.header.stamp.sec + 1e-9 * p.header.stamp.nanosec;
        auto dt = current_time - ped_time;
        // don't consider this pedestrian if it came in too long ago.
        if (dt > people_timeout){
          RCLCPP_INFO_STREAM(get_logger(), "Ignoring pedestrian " << p.id);
          continue;
        }
        RCLCPP_INFO_STREAM(get_logger(), "pedestrian ID: " << p.id <<
                                          "\n\tPosition: " << p.pose.position.x << " " << p.pose.position.y <<
                                          "\n\tVelocity: " << p.velocity.linear.x << " " << p.velocity.linear.y <<
                                          "\nTimestamp: " << p.header.stamp.sec  << "s " << p.header.stamp.nanosec << " ns");
        RCLCPP_INFO_STREAM(get_logger(), "Time Difference " << dt);

        // compute distance to the pedestrian from the robot
        auto dist_to_ped = dist(robot_pose.x, robot_pose.y, p.pose.position.x, p.pose.position.y);
        if (dist_to_ped > brne_activate_threshold){
          RCLCPP_INFO_STREAM(get_logger(), "Pedestrian " << p.id << " too far away");
          continue;
        }
        dists_to_peds.push_back(dist_to_ped);
        selected_peds.pedestrians.push_back(p);
      }

      auto n_peds = static_cast<int>(selected_peds.pedestrians.size());
      auto n_agents = std::min(maximum_agents, n_peds + 1);

      RCLCPP_INFO_STREAM(get_logger(), "Agents: " << n_agents);

      arma::rowvec goal_vec;
      if (goal_set){
        goal_vec = arma::rowvec(std::vector<double>{goal.pose.position.x,
                                                    goal.pose.position.y});
      } else {
        goal_vec = arma::rowvec(std::vector<double>{6.0, 0.0});
      }

      // get the controls to go to the goal
      auto theta_a = robot_pose.theta;
      if (robot_pose.theta > 0.0){
        theta_a -= M_PI_2;
      } else {
        theta_a += M_PI_2;
      }
      arma::rowvec axis_vec(std::vector<double>{cos(theta_a), sin(theta_a)});
      arma::rowvec pose_vec(std::vector<double>{robot_pose.x, robot_pose.y});
      arma::rowvec vec_to_goal = goal_vec - pose_vec;
      auto dist_to_goal = arma::norm(vec_to_goal);
      auto proj_len = arma::dot(axis_vec, vec_to_goal) / arma::dot(vec_to_goal, vec_to_goal) * dist_to_goal;
      auto radius = 0.5 * dist_to_goal / proj_len;
      // find nominal linear and angular velocity
      double nominal_ang_vel = 0;
      if (robot_pose.theta > 0.0){
        nominal_ang_vel = -nominal_lin_vel/radius;
      } else {
        nominal_ang_vel =  nominal_lin_vel/radius;
      }
      auto traj_samples = trajgen.traj_sample(nominal_lin_vel, nominal_ang_vel, robot_pose.toVec());


      if (n_agents > 1){
        // pick only the closest pedestrians to interact with
        auto robot_xtraj_samples = trajgen.get_xtraj_samples();
        auto robot_ytraj_samples = trajgen.get_ytraj_samples();

      } else {
        // go straight to the goal
        auto opt_cmds = trajgen.opt_controls(goal_vec);
        // RCLCPP_INFO_STREAM(get_logger(), "Opt cmds\n" << opt_cmds);
        if (goal_set){
          for (int i=0; i<n_steps; i++){
            geometry_msgs::msg::Twist tw;
            tw.linear.x = opt_cmds.at(i,0);
            tw.angular.z = opt_cmds.at(i,1);
            robot_cmds.twists.push_back(tw);
          }
        }
        // compute the optimal path
        optimal_path.header.stamp = current_timestamp;
        optimal_path.header.frame_id = "odom";
        optimal_path.poses.clear();
        for (int i=0; i<n_steps; i++){
          geometry_msgs::msg::PoseStamped ps;
          ps.header.stamp = current_timestamp;
          ps.header.frame_id = "odom";
          // TODO fill these with the actual trajectory
          ps.pose.position.x = i*0.1;
          ps.pose.position.y = 0;
          optimal_path.poses.push_back(ps);
        }

      }
      // publish controls
      cmd_buf_pub_->publish(robot_cmds);
      // publish optimal path
      path_pub_->publish(optimal_path);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlan>());
  rclcpp::shutdown();
  return 0;
}