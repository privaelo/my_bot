// Copyright 2026 privaelo
//
// Constant-velocity prediction of the moving target's pose. Transforms
// ground-truth odometry (odom, frame target/odom) into the map frame,
// rotates the body-frame twist into map frame by the current heading, and
// extrapolates position prediction_horizon seconds ahead. Reactive on every
// odom message; no timer, no wall-clock reads.

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class TargetPredictor : public rclcpp::Node
{
public:
  TargetPredictor()
  : Node("target_predictor"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    prediction_horizon_ = declare_parameter("prediction_horizon", 1.0);
    map_frame_ = declare_parameter("map_frame", "map");

    RCLCPP_INFO(
      get_logger(), "prediction_horizon %.2f s, map_frame '%s'",
      prediction_horizon_, map_frame_.c_str());

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("predicted_pose", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {on_odom(*msg);});
  }

private:
  void on_odom(const nav_msgs::msg::Odometry & odom)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    try {
      pose = tf_buffer_.transform(pose, map_frame_, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "tf unavailable: %s", ex.what());
      return;
    }

    const double yaw = tf2::getYaw(pose.pose.orientation);
    const double vx_body = odom.twist.twist.linear.x;
    const double vy_body = odom.twist.twist.linear.y;
    const double vx_map = vx_body * std::cos(yaw) - vy_body * std::sin(yaw);
    const double vy_map = vx_body * std::sin(yaw) + vy_body * std::cos(yaw);

    geometry_msgs::msg::PoseStamped predicted;
    predicted.header.frame_id = map_frame_;
    predicted.header.stamp = rclcpp::Time(pose.header.stamp) +
      rclcpp::Duration::from_seconds(prediction_horizon_);
    predicted.pose.position.x = pose.pose.position.x + vx_map * prediction_horizon_;
    predicted.pose.position.y = pose.pose.position.y + vy_map * prediction_horizon_;
    predicted.pose.orientation = pose.pose.orientation;
    pose_pub_->publish(predicted);
  }

  double prediction_horizon_{};
  std::string map_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetPredictor>());
  rclcpp::shutdown();
  return 0;
}
