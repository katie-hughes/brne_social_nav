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
#include <opencv2/highgui.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

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

      people_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("people", 10);

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

      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    }

  private:
    double rate_hz;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr people_pub_;

    rclcpp::Subscription<unitree_crowd_nav_interfaces::msg::PixelArray>::SharedPtr pixel_left_sub_;
    rclcpp::Subscription<unitree_crowd_nav_interfaces::msg::PixelArray>::SharedPtr pixel_right_sub_;

    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_right_sub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

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
        const auto success = stereo.fromCameraInfo(info_left, info_right);
        RCLCPP_INFO_STREAM(get_logger(), "Initialized camera: Success? "<<success);
      }
      // in here I should analyze the pixels that come in and try to match them up.
      if (stereo.initialized()){
        // match them up 
        // naive approach: Right now I only have one person walking around. 
        // so I just check if there is one point in left and one point in right.
        // if there is I just assume these two are from the same image!!
        if ((left_pixels.pixels.size() == right_pixels.pixels.size()) 
            && left_pixels.pixels.size() > 0){
          // visualization_msgs::msg::MarkerArray ma;
          for (int i = 0; i < static_cast<int>(left_pixels.pixels.size()); i++){
            RCLCPP_INFO_STREAM(get_logger(), "L: ("<<left_pixels.pixels.at(i).x << ","<<left_pixels.pixels.at(i).y<<")"<<
                                            " R: ("<<right_pixels.pixels.at(i).x << ","<<right_pixels.pixels.at(i).y<<")");
            // compute disparity: difference in x (left - right)
            const auto disparity = left_pixels.pixels.at(i).x - right_pixels.pixels.at(i).x;
            RCLCPP_INFO_STREAM(get_logger(), "Disparity: " << disparity);
            // calculate 3d point
            // use this function: 
            // projectDisparityTo3d (const cv::Point2d &left_uv_rect, float disparity, cv::Point3d &xyz) const
            cv::Point2d left_uv_rect(left_pixels.pixels.at(i).x, left_pixels.pixels.at(i).y);
            // RCLCPP_INFO_STREAM(get_logger(), "leftuv: " << left_uv_rect.x << "  "<< left_uv_rect.y);
            cv::Point3d xyz;
            stereo.projectDisparityTo3d(left_uv_rect, disparity, xyz);
            // print out person location
            RCLCPP_INFO_STREAM(get_logger(), "xyz: (" << xyz.x << ", "<< xyz.y << ", "<<xyz.z<<")");

            geometry_msgs::msg::TransformStamped T_person;
            T_person.header.frame_id = "camera_face";
            T_person.child_frame_id = "person1";
            T_person.header.stamp = this->get_clock()->now();
            // on the unitree, x faces forward from camera. Y to the right. Z down. 
            // (tf: x: red, y: green, z: blue)
            // I think this gives to me with z out from camera. 
            T_person.transform.translation.x = xyz.z;
            T_person.transform.translation.y = xyz.x;
            T_person.transform.translation.z = xyz.y;
            tf_broadcaster_->sendTransform(T_person);

            // // publish markers
            // visualization_msgs::msg::Marker m;
            // m.header.stamp = this->get_clock()->now();
            // m.header.frame_id = "camera_face";
            // m.id = i;         // so each has a unique ID
            // m.type = 3;       // cylinder
            // // Set color as yellow
            // m.color.r = 1.0;
            // m.color.g = 1.0;
            // m.color.b = 0.0;
            // m.color.a = 1.0;
            // // Set Radius
            // m.scale.x = 2 * 0.4;
            // m.scale.y = 2 * 0.4;
            // m.scale.z = 1.0;
            // // set position
            // m.pose.position.x = 0; // xyz.x;
            // m.pose.position.y = 0; // xyz.y;
            // m.pose.position.z = xyz.z;
            // // Add to marker array
            // ma.markers.push_back(m);
            }
          // people_pub_->publish(ma);
        }
      }
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrackPedestrians>());
  rclcpp::shutdown();
  return 0;
}