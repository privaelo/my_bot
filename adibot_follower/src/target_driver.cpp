// Copyright 2026 privaelo
//
// Drives the target model around a waypoint circuit.
//
// Subscribes ground-truth odometry (odom, frame target/odom), transforms it
// to the map frame via tf, and publishes unicycle velocity commands (cmd_vel)
// that the model's VelocityControl plugin executes. Waypoints are map-frame.

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class TargetDriver : public rclcpp::Node
{
public:
  TargetDriver()
  : Node("target_driver"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    speed_ = declare_parameter("speed", 0.5);
    loop_ = declare_parameter("loop", true);
    goal_tolerance_ = declare_parameter("goal_tolerance", 0.3);
    k_heading_ = declare_parameter("k_heading", 1.5);
    max_angular_ = declare_parameter("max_angular", 1.5);
    slow_angle_ = declare_parameter("slow_angle", 0.8);
    map_frame_ = declare_parameter("map_frame", "map");
    const double rate = declare_parameter("control_rate", 20.0);
    const auto flat = declare_parameter(
      "waypoints", std::vector<double>{6.0, 6.0, -6.0, 6.0, -6.0, -6.0, 6.0, -6.0});

    if (flat.size() < 4 || flat.size() % 2 != 0) {
      throw std::invalid_argument("waypoints must be a flat [x0,y0,x1,y1,...] list, >= 2 points");
    }
    for (size_t i = 0; i < flat.size(); i += 2) {
      waypoints_.emplace_back(flat[i], flat[i + 1]);
    }

    RCLCPP_INFO(
      get_logger(), "%zu waypoints, speed %.2f m/s, loop %s, tolerance %.2f m, rate %.1f Hz",
      waypoints_.size(), speed_, loop_ ? "on" : "off", goal_tolerance_, rate);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {odom_ = std::move(msg);});
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate), [this] {tick();});
  }

private:
  void tick()
  {
    if (done_) {
      return;
    }
    if (!odom_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for odometry");
      return;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom_->header;
    pose.pose = odom_->pose.pose;
    try {
      // static map -> target/odom transform published by the launch file
      pose = tf_buffer_.transform(pose, map_frame_, tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "tf unavailable: %s", ex.what());
      return;
    }

    const double x = pose.pose.position.x;
    const double y = pose.pose.position.y;
    double dx = waypoints_[index_].first - x;
    double dy = waypoints_[index_].second - y;
    if (std::hypot(dx, dy) < goal_tolerance_) {
      if (index_ + 1 == waypoints_.size() && !loop_) {
        cmd_pub_->publish(geometry_msgs::msg::Twist());
        done_ = true;
        RCLCPP_INFO(get_logger(), "final waypoint reached, stopping");
        return;
      }
      index_ = (index_ + 1) % waypoints_.size();
      RCLCPP_INFO(
        get_logger(), "heading to waypoint %zu (%.1f, %.1f)",
        index_, waypoints_[index_].first, waypoints_[index_].second);
      dx = waypoints_[index_].first - x;
      dy = waypoints_[index_].second - y;
    }

    const double yaw = tf2::getYaw(pose.pose.orientation);
    const double heading_error = std::atan2(
      std::sin(std::atan2(dy, dx) - yaw), std::cos(std::atan2(dy, dx) - yaw));

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(k_heading_ * heading_error, -max_angular_, max_angular_);
    // turn in place until roughly facing the waypoint
    cmd.linear.x = std::abs(heading_error) < slow_angle_ ? speed_ : 0.0;
    cmd_pub_->publish(cmd);
  }

  std::vector<std::pair<double, double>> waypoints_;
  double speed_{}, goal_tolerance_{}, k_heading_{}, max_angular_{}, slow_angle_{};
  bool loop_{};
  std::string map_frame_;
  size_t index_{0};
  bool done_{false};
  nav_msgs::msg::Odometry::SharedPtr odom_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetDriver>());
  rclcpp::shutdown();
  return 0;
}
