#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

class OdomPassthrough : public rclcpp::Node
{
  public:
    OdomPassthrough()
    : Node("odom_passthrough")
    {

      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/zed2i/zed_node/odom", 10, std::bind(&OdomPassthrough::odom_cb, this, std::placeholders::_1));
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    }

  private:

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    void odom_cb(const nav_msgs::msg::Odometry & msg)
    {
      odom_pub_->publish(msg);
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPassthrough>());
  rclcpp::shutdown();
  return 0;
}