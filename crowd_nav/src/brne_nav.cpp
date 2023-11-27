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
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

struct RobotPose
{
  double x;
  double y;
  double theta;
  arma::rowvec toVec()
  {
    return arma::rowvec(std::vector<double>{x, y, theta});
  }
};

double dist(double x1, double y1, double x2, double y2)
{
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

class PathPlan : public rclcpp::Node
{
public:
  PathPlan()
  : Node("brne"), goal_set{false}, walls_published{false}
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
    declare_parameter("close_stop_threshold", 1.0);

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
    close_stop_threshold = get_parameter("close_stop_threshold").as_double();

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
    RCLCPP_INFO_STREAM(get_logger(), "Close stop threshold " << close_stop_threshold << "m");
    RCLCPP_INFO_STREAM(get_logger(), "Brne Activate Threshold " << brne_activate_threshold << "m");
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Max Lin: " << max_lin_vel << " nominal lin: " << nominal_lin_vel << " max ang: " <<
        max_ang_vel);

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
      "brne_odom", 10, std::bind(&PathPlan::odom_cb, this, std::placeholders::_1));
    cmd_buf_pub_ = create_publisher<crowd_nav_interfaces::msg::TwistArray>("cmd_buf", 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/optimal_path", 10);
    walls_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/walls", 10);
    brne_peds_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/brne_peds", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create a timer that executes at replan_freq
    std::chrono::milliseconds rate = (std::chrono::milliseconds) ((int)(1000. / replan_freq));
    timer_ = create_wall_timer(rate, std::bind(&PathPlan::timer_callback, this));

    optimal_path.header.frame_id = "brne_odom";
  }

private:
  double replan_freq, kernel_a1, kernel_a2, cost_a1, cost_a2, cost_a3, y_min, y_max, dt,
    max_ang_vel, max_lin_vel, people_timeout, goal_threshold, brne_activate_threshold,
    nominal_lin_vel, close_stop_threshold;
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

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<crowd_nav_interfaces::msg::PedestrianArray>::SharedPtr pedestrian_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<crowd_nav_interfaces::msg::TwistArray>::SharedPtr cmd_buf_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr walls_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr brne_peds_pub_;

  bool goal_set;
  bool walls_published;
  geometry_msgs::msg::PoseStamped goal;

  rclcpp::Time last_ped_stamp;
  rclcpp::Time curr_ped_stamp;

  void pub_walls()
  {
    auto now = this->get_clock()->now();
    auto height = 1.0;
    auto length = 10.0;
    auto thickness = 0.01;
    auto transparency = 0.2;
    visualization_msgs::msg::MarkerArray ma;
    for (int i = 0; i < 2; i++) {
      visualization_msgs::msg::Marker wall;
      wall.header.frame_id = "brne_odom";
      wall.header.stamp = now;
      wall.id = i;
      wall.type = 1;   // cube
      wall.action = 0;
      wall.pose.position.x = 0.5 * length - 1.0;
      wall.pose.position.y = y_min;
      wall.pose.position.z = 0.5 * height;
      wall.color.a = transparency;
      wall.color.b = 1.0;
      wall.scale.x = length;
      wall.scale.y = thickness;
      wall.scale.z = height;
      ma.markers.push_back(wall);
    }
    ma.markers.at(0).pose.position.y = y_min;
    ma.markers.at(0).pose.position.y = y_max;
    walls_pub_->publish(ma);
  }

  void goal_cb(const geometry_msgs::msg::PoseStamped & msg)
  {
    RCLCPP_INFO_STREAM(
      get_logger(), "Goal Received: " << msg.pose.position.x << ", " << msg.pose.position.y);
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
    if (goal_set) {
      check_goal();
    }
  }

  void check_goal()
  {
    auto dist_to_goal =
      dist(robot_pose.x, robot_pose.y, goal.pose.position.x, goal.pose.position.y);
    if (dist_to_goal < goal_threshold) {
      RCLCPP_INFO_STREAM(get_logger(), "Goal Reached!");
      goal_set = false;
    }
  }

  void pedestrians_cb(const crowd_nav_interfaces::msg::PedestrianArray & msg)
  {
    curr_ped_stamp = this->get_clock()->now();
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
    for (int p = 0; p < n_peds; p++) {
      auto ped = peds.pedestrians.at(p);
      // RCLCPP_INFO_STREAM(get_logger(), "Ped: " << ped.pose.position.x << "," << ped.pose.position.y);
      ped_array.at(p, 0) = ped.pose.position.x;
      ped_array.at(p, 1) = ped.pose.position.y;
      // make a matrix that looks like
      // x y
      // x y
      // ... number of rows = number of pedestrians.
      arma::rowvec f2f_vel(2, arma::fill::zeros);
      if (n_prev_peds > 0) {
        arma::vec xs(n_prev_peds, arma::fill::value(ped.pose.position.x));
        arma::vec ys(n_prev_peds, arma::fill::value(ped.pose.position.y));
        for_compare.col(0) = xs;
        for_compare.col(1) = ys;
        arma::mat difference = prev_ped_array - for_compare;
        auto norm = arma::vecnorm(difference, 2, 1);
        f2f_vel = ped_array.row(p) - prev_ped_array.row(norm.index_min());
        f2f_vel /= (curr_ped_stamp - last_ped_stamp).seconds();
        // RCLCPP_INFO_STREAM(get_logger(), "f2f vel: " << f2f_vel);
      }
      peds.pedestrians.at(p).velocity.linear.x = f2f_vel.at(0);
      peds.pedestrians.at(p).velocity.linear.y = f2f_vel.at(1);
      // add to pedestrian buffer
      // first have to do some error checking to make sure the index is in bounds
      // initially length is 0 and I want to append pedestrian 0
      // while(0<1)go through
      // while(1<1)BREAK
      // try this to fix time synnchronization issues
      peds.pedestrians.at(p).header.stamp = curr_ped_stamp;
      while (static_cast<int>(ped_buffer.pedestrians.size()) < static_cast<int>(ped.id + 1)) {
        crowd_nav_interfaces::msg::Pedestrian blank_ped;
        blank_ped.id = ped_buffer.pedestrians.size();
        ped_buffer.pedestrians.push_back(blank_ped);
      }
      ped_buffer.pedestrians.at(ped.id) = peds.pedestrians.at(p);
    }
    last_ped_stamp = curr_ped_stamp;
    // RCLCPP_INFO_STREAM(get_logger(), "ped array\n" << ped_array);
  }

  void pub_ped_markers(){
    // plot selected peds
    const auto now = this->get_clock()->now();
    const auto thickness = 0.05;
    const auto transparency = 0.5;
    const auto velocity_scale = 1.0;
    const auto base_vel = 0.05;
    visualization_msgs::msg::MarkerArray ma;
    for (int i=0; i<static_cast<int>(selected_peds.pedestrians.size()); i++){
      const auto pose_i = selected_peds.pedestrians.at(i).pose.position;
      const auto vel_i = selected_peds.pedestrians.at(i).velocity.linear;
      const auto vel_abs = dist(0, 0, vel_i.x, vel_i.y);
      // create an arrow marker
      visualization_msgs::msg::Marker ped_i;
      ped_i.header.frame_id = "brne_odom";
      ped_i.header.stamp = now;
      ped_i.id = i;
      ped_i.type = 0;   // arrow
      ped_i.action = 0;
      // pose is the end of the arrow
      ped_i.pose.position.x = pose_i.x;
      ped_i.pose.position.y = pose_i.y;
      ped_i.pose.position.z = 0.01;
      // orientation should be the velocity direction
      const auto phi = atan2(vel_i.y, vel_i.x);
      tf2::Quaternion q;
      q.setRPY(0, 0, phi);
      ped_i.pose.orientation.w = q.w();
      ped_i.pose.orientation.x = q.x();
      ped_i.pose.orientation.y = q.y();
      ped_i.pose.orientation.z = q.z();
      // scale is the size of the arrow, proportional to velocity
      ped_i.scale.x = velocity_scale*vel_abs + base_vel;
      ped_i.scale.y = thickness;
      ped_i.scale.z = 0.01;
      ped_i.color.a = transparency;
      ped_i.color.r = 1.0;
      ped_i.color.g = 0.67;
      ped_i.color.b = 0.0;
      ped_i.lifetime.sec = 1;
      ma.markers.push_back(ped_i);
    }
    brne_peds_pub_->publish(ma);
  }

  void timer_callback()
  {
    auto start = this->get_clock()->now();
    robot_cmds.twists.clear();
    // RCLCPP_INFO_STREAM(get_logger(), "\n\nTimer Tick");
    // RCLCPP_INFO_STREAM(get_logger(), "Robot @: " << robot_pose.x << " " << robot_pose.y << " " << robot_pose.theta);
    // read in pedestrian buffer
    builtin_interfaces::msg::Time current_timestamp;
    current_timestamp = this->get_clock()->now();
    auto current_time = current_timestamp.sec + 1e-9 * current_timestamp.nanosec;
    // RCLCPP_INFO_STREAM(get_logger(), "Current time: " << current_timestamp.sec << "s " << current_timestamp.nanosec << "ns ");

    selected_peds.pedestrians.clear();
    std::vector<double> dists_to_peds;
    for (auto p:ped_buffer.pedestrians) {
      auto ped_time = p.header.stamp.sec + 1e-9 * p.header.stamp.nanosec;
      auto dt = current_time - ped_time;
      // don't consider this pedestrian if it came in too long ago.
      if (dt > people_timeout) {
        // RCLCPP_INFO_STREAM(get_logger(), "Ignoring pedestrian " << p.id);
        continue;
      }
      // compute distance to the pedestrian from the robot
      auto dist_to_ped = dist(robot_pose.x, robot_pose.y, p.pose.position.x, p.pose.position.y);
      if (dist_to_ped > brne_activate_threshold) {
        // RCLCPP_INFO_STREAM(get_logger(), "Pedestrian " << p.id << " too far away");
        continue;
      }
      dists_to_peds.push_back(dist_to_ped);
      selected_peds.pedestrians.push_back(p);
    }

    auto n_peds = static_cast<int>(selected_peds.pedestrians.size());
    auto n_agents = std::min(maximum_agents, n_peds + 1);

    pub_ped_markers();

    // RCLCPP_DEBUG_STREAM(get_logger(), "Agents: " << n_agents);

    arma::rowvec goal_vec;
    if (goal_set) {
      goal_vec = arma::rowvec(
        std::vector<double>{goal.pose.position.x,
          goal.pose.position.y});
    } else {
      goal_vec = arma::rowvec(std::vector<double>{6.0, 0.0});
    }

    // get the controls to go to the goal
    auto theta_a = robot_pose.theta;
    if (robot_pose.theta > 0.0) {
      theta_a -= M_PI_2;
    } else {
      theta_a += M_PI_2;
    }
    arma::rowvec axis_vec(std::vector<double>{cos(theta_a), sin(theta_a)});
    arma::rowvec pose_vec(std::vector<double>{robot_pose.x, robot_pose.y});
    arma::rowvec vec_to_goal = goal_vec - pose_vec;
    auto dist_to_goal = arma::norm(vec_to_goal);
    auto proj_len =
      arma::dot(axis_vec, vec_to_goal) / arma::dot(vec_to_goal, vec_to_goal) * dist_to_goal;
    auto radius = 0.5 * dist_to_goal / proj_len;
    // find nominal linear and angular velocity
    double nominal_ang_vel = 0;
    if (robot_pose.theta > 0.0) {
      nominal_ang_vel = -nominal_lin_vel / radius;
    } else {
      nominal_ang_vel = nominal_lin_vel / radius;
    }
    auto traj_samples = trajgen.traj_sample(nominal_lin_vel, nominal_ang_vel, robot_pose.toVec());


    if (n_agents > 1) {
      // create pedestrian samples
      auto x_pts = brne.mvn_sample_normal(n_agents - 1);
      auto y_pts = brne.mvn_sample_normal(n_agents - 1);

      // these have been filled in
      arma::mat xtraj_samples(n_agents * n_samples, n_steps, arma::fill::zeros);
      arma::mat ytraj_samples(n_agents * n_samples, n_steps, arma::fill::zeros);

      // pick only the closest pedestrians to interact with
      // dists_to_peds and selected_peds arrays
      auto closest_idxs =
        arma::conv_to<arma::vec>::from(arma::sort_index(arma::vec(dists_to_peds)));
      for (int p = 0; p < (n_agents - 1); p++) {
        auto ped = selected_peds.pedestrians.at(closest_idxs.at(p));
        // RCLCPP_INFO_STREAM(get_logger(), "Ped " << ped.id << " pos " <<
        //                                  ped.pose.position.x << " " <<
        //                                  ped.pose.position.y);
        // speed factor
        arma::vec ped_vel(std::vector<double>{ped.velocity.linear.x, ped.velocity.linear.y});
        auto speed_factor = arma::norm(ped_vel);
        arma::rowvec ped_xmean = arma::rowvec(n_steps, arma::fill::value(ped.pose.position.x)) +
          arma::linspace<arma::rowvec>(0, (n_steps - 1), n_steps) * dt * ped.velocity.linear.x;
        arma::rowvec ped_ymean = arma::rowvec(n_steps, arma::fill::value(ped.pose.position.y)) +
          arma::linspace<arma::rowvec>(0, (n_steps - 1), n_steps) * dt * ped.velocity.linear.y;
        arma::mat ped_xmean_mat(n_samples, n_steps, arma::fill::zeros);
        arma::mat ped_ymean_mat(n_samples, n_steps, arma::fill::zeros);
        ped_xmean_mat.each_row() = ped_xmean;
        ped_ymean_mat.each_row() = ped_ymean;
        // RCLCPP_INFO_STREAM(get_logger(), "Ped xmean mat\n" << ped_xmean_mat);
        // set submatrix in xtraj and ytraj samples.
        // submatrix ((p+1)*nsamples, (p+2)*nsamples) = xpoints(p*nsamples, (p+1)*nsamples) * speed_factor + ped_xmean
        xtraj_samples.submat((p + 1) * n_samples, 0, (p + 2) * n_samples - 1, n_steps - 1) =
          x_pts.submat(
          p * n_samples, 0, (p + 1) * n_samples - 1,
          n_steps - 1) * speed_factor + ped_xmean_mat;
        ytraj_samples.submat((p + 1) * n_samples, 0, (p + 2) * n_samples - 1, n_steps - 1) =
          y_pts.submat(
          p * n_samples, 0, (p + 1) * n_samples - 1,
          n_steps - 1) * speed_factor + ped_ymean_mat;
        // if the speed factor is 0 then this will just be equal to ped_xmean
      }
      // apply the robot's samples
      auto robot_xtraj_samples = trajgen.get_xtraj_samples();
      auto robot_ytraj_samples = trajgen.get_ytraj_samples();
      xtraj_samples.submat(0, 0, n_samples - 1, n_steps - 1) = robot_xtraj_samples;
      ytraj_samples.submat(0, 0, n_samples - 1, n_steps - 1) = robot_ytraj_samples;
      // after this xtraj and ytraj samples are fully filled in!

      // Safety mask calculation
      // the idea is to see if the distance to the closest pedestrian
      // in any of the robot samples is less than the close stop threshold
      auto closest_ped = selected_peds.pedestrians.at(closest_idxs.at(0));
      arma::mat robot_samples_to_ped = arma::sqrt(
        arma::pow(
          robot_xtraj_samples -
          closest_ped.pose.position.x, 2) +
        arma::pow(
          robot_ytraj_samples -
          closest_ped.pose.position.y, 2));
      auto closest_to_ped = arma::conv_to<arma::vec>::from(arma::min(robot_samples_to_ped, 1));
      auto safety_mask = arma::conv_to<arma::rowvec>::from(closest_to_ped > close_stop_threshold);
      // RCLCPP_INFO_STREAM(get_logger(), "percent of safe samples" << arma::mean(safety_mask) * 100.0);

      // BRNE OPTIMIZATION HERE
      // const auto weights_start = this->get_clock()->now();
      auto weights = brne.brne_nav(xtraj_samples, ytraj_samples);
      // const auto weights_end = this->get_clock()->now();
      // const auto weights_diff = weights_end - weights_start;
      // RCLCPP_DEBUG_STREAM(get_logger(), "Weights calculation: " << weights_diff.seconds() << " s");

      // apply the safety mask to the weights for the robot
      weights.row(0) %= safety_mask;
      const double mean_weights = arma::mean(weights.row(0));
      if (mean_weights != 0) {
        weights.row(0) /= mean_weights;
      } else {
        RCLCPP_INFO_STREAM(get_logger(), "E-Stop from safety mask!");
      }

      const auto ulist = trajgen.get_ulist();
      const auto ulist_lin = arma::conv_to<arma::rowvec>::from(ulist.col(0));
      const auto ulist_ang = arma::conv_to<arma::rowvec>::from(ulist.col(1));
      const auto opt_cmds_lin = arma::mean(ulist_lin % weights.row(0));
      const auto opt_cmds_ang = arma::mean(ulist_ang % weights.row(0));

      if (goal_set) {
        for (int i = 0; i < n_steps; i++) {
          geometry_msgs::msg::Twist tw;
          tw.linear.x = opt_cmds_lin;
          tw.angular.z = opt_cmds_ang;
          // manually adjust for dog's drift (discovered these values experimentally)
          if ((opt_cmds_lin > 0.1) && (opt_cmds_lin < 0.3)){
            tw.angular.z -= 0.04;
          } else if ((opt_cmds_lin >= 0.3) && (opt_cmds_lin < 0.5)){
            tw.angular.z -= 0.05;
          } else if (opt_cmds_lin >= 0.5){
            tw.angular.z -= 0.06;
          }
          if (i == 0){
            RCLCPP_INFO_STREAM(get_logger(), "Opt lin " << opt_cmds_lin << " ang " << opt_cmds_ang 
                                          << "Twist lin: " << tw.linear.x << " ang " << tw.angular.z);
          }
          robot_cmds.twists.push_back(tw);
        }
      }

      arma::mat opt_cmds(n_steps, 2, arma::fill::zeros);
      opt_cmds.col(0) = arma::vec(n_steps, arma::fill::value(opt_cmds_lin));
      opt_cmds.col(1) = arma::vec(n_steps, arma::fill::value(opt_cmds_ang));

      // RCLCPP_INFO_STREAM(get_logger(), "Opt cmds\n" << opt_cmds);

      // compute the optimal path
      const auto opt_traj = trajgen.sim_traj(robot_pose.toVec(), opt_cmds);
      optimal_path.header.stamp = current_timestamp;
      optimal_path.poses.clear();
      for (int i = 0; i < n_steps; i++) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = current_timestamp;
        ps.header.frame_id = "brne_odom";
        ps.pose.position.x = opt_traj.at(i, 0);
        ps.pose.position.y = opt_traj.at(i, 1);
        optimal_path.poses.push_back(ps);
      }


    } else {
      // go straight to the goal
      const auto opt_cmds = trajgen.opt_controls(goal_vec);
      if (goal_set) {
        for (int i = 0; i < n_steps; i++) {
          geometry_msgs::msg::Twist tw;
          const auto opt_cmds_lin = opt_cmds.at(i, 0);
          const auto opt_cmds_ang = opt_cmds.at(i, 1);
          tw.linear.x = opt_cmds_lin;
          tw.angular.z = opt_cmds_ang;
          // manually adjust for dog's drift (discovered these values experimentally)
          if ((opt_cmds_lin > 0.1) && (opt_cmds_lin < 0.3)){
            tw.angular.z -= 0.04;
          } else if ((opt_cmds_lin >= 0.3) && (opt_cmds_lin < 0.5)){
            tw.angular.z -= 0.05;
          } else if (opt_cmds_lin >= 0.5){
            tw.angular.z -= 0.06;
          }
          if (i == 0){
            RCLCPP_INFO_STREAM(get_logger(), "Opt lin " << opt_cmds_lin << " ang " << opt_cmds_ang 
                                          << "Twist lin: " << tw.linear.x << " ang " << tw.angular.z);
          }
          robot_cmds.twists.push_back(tw);
        }
      }
      // compute the optimal path
      const auto opt_traj = trajgen.sim_traj(robot_pose.toVec(), opt_cmds);
      optimal_path.header.stamp = current_timestamp;
      optimal_path.poses.clear();
      for (int i = 0; i < n_steps; i++) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = current_timestamp;
        ps.header.frame_id = "brne_odom";
        ps.pose.position.x = opt_traj.at(i, 0);
        ps.pose.position.y = opt_traj.at(i, 1);
        optimal_path.poses.push_back(ps);
      }
    }
    // publish controls
    cmd_buf_pub_->publish(robot_cmds);
    // publish optimal path
    path_pub_->publish(optimal_path);
    // publish the walls
    pub_walls();
    // if (!walls_published){
    //   pub_walls();
    //   walls_published = true;
    // }

    auto end = this->get_clock()->now();
    auto diff = end - start;
    RCLCPP_DEBUG_STREAM(get_logger(), "Agents: " << n_agents << " Timer: " << diff.seconds() << " s");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPlan>());
  rclcpp::shutdown();
  return 0;
}
