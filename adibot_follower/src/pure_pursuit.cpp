// Copyright 2026 privaelo
//
// Pure-pursuit path tracking: turns /plan into cmd_vel. Tapers speed to zero
// standoff_distance from the path's end (the target's predicted position) so
// the follower trails instead of ramming it. Geometry lives in
// pure_pursuit.hpp; this node is just the ROS glue.
//
// Unlike target_driver/global_planner, every degraded branch here explicitly
// publishes a zero TwistStamped instead of skipping the tick: gz's DiffDrive
// plugin keeps executing the last cmd_vel forever if nothing new arrives, so
// silently skipping would leave the robot coasting on a stale command.

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "adibot_follower/pure_pursuit.hpp"

using namespace std::chrono_literals;

class PurePursuit : public rclcpp::Node
{
public:
  PurePursuit()
  : Node("pure_pursuit"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    lookahead_distance_ = declare_parameter("lookahead_distance", 0.6);
    standoff_distance_ = declare_parameter("standoff_distance", 1.0);
    max_linear_ = declare_parameter("max_linear", 0.8);
    max_angular_ = declare_parameter("max_angular", 1.5);
    map_frame_ = declare_parameter("map_frame", "map");
    robot_frame_ = declare_parameter("robot_frame", "base_link");
    const double rate = declare_parameter("control_rate", 20.0);

    RCLCPP_INFO(
      get_logger(),
      "lookahead_distance %.2f m, standoff_distance %.2f m, max_linear %.2f m/s, "
      "max_angular %.2f rad/s, control_rate %.1f Hz, map_frame '%s', robot_frame '%s'",
      lookahead_distance_, standoff_distance_, max_linear_, max_angular_, rate,
      map_frame_.c_str(), robot_frame_.c_str());

    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "plan", 10, [this](nav_msgs::msg::Path::SharedPtr msg) {on_path(*msg);});
    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / rate), [this] {tick();});
  }

private:
  void on_path(const nav_msgs::msg::Path & msg)
  {
    path_.clear();
    path_.reserve(msg.poses.size());
    for (const auto & pose : msg.poses) {
      path_.push_back({pose.pose.position.x, pose.pose.position.y});
    }
  }

  void publish_zero()
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.frame_id = robot_frame_;
    cmd.header.stamp = now();
    cmd_pub_->publish(cmd);
  }

  void tick()
  {
    if (path_.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for a path");
      publish_zero();
      return;
    }

    geometry_msgs::msg::TransformStamped robot_tf;
    try {
      robot_tf = tf_buffer_.lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "tf unavailable: %s", ex.what());
      publish_zero();
      return;
    }

    const adibot_follower::Pose2D robot{
      robot_tf.transform.translation.x, robot_tf.transform.translation.y,
      tf2::getYaw(robot_tf.transform.rotation)};

    const auto & end = path_.back();
    const double dist_to_end = std::hypot(end.x - robot.x, end.y - robot.y);
    const double speed_scale = std::clamp(
      (dist_to_end - standoff_distance_) / lookahead_distance_, 0.0, 1.0);
    if (speed_scale <= 0.0) {
      publish_zero();
      return;
    }

    const auto lookahead = adibot_follower::find_lookahead_point(
      path_, robot, lookahead_distance_);
    if (!lookahead) {
      publish_zero();
      return;
    }

    const double curvature = adibot_follower::curvature_to_point(robot, *lookahead);
    // Slow into sharp turns so the commanded turn stays achievable. At full
    // speed the tightest turn is max_linear/max_angular (~0.53 m radius); where
    // the A* path curls tighter the robot would understeer and overshoot into
    // an obstacle. Capping linear at max_angular/|curvature| holds the turn rate
    // at max_angular while creeping forward, so a diff-drive can actually track
    // the curve (degrades to a near-in-place turn for very sharp kinks).
    double linear = max_linear_ * speed_scale;
    if (std::abs(curvature) > 1e-3) {
      linear = std::min(linear, max_angular_ / std::abs(curvature));
    }
    const double angular = std::clamp(curvature * linear, -max_angular_, max_angular_);

    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.frame_id = robot_frame_;
    cmd.header.stamp = now();
    cmd.twist.linear.x = linear;
    cmd.twist.angular.z = angular;
    cmd_pub_->publish(cmd);
  }

  double lookahead_distance_{}, standoff_distance_{}, max_linear_{}, max_angular_{};
  std::string map_frame_, robot_frame_;
  std::vector<adibot_follower::Point2D> path_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuit>());
  rclcpp::shutdown();
  return 0;
}
