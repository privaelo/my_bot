// Copyright 2026 privaelo
//
// Replans an A* path at a fixed rate from the robot's tf pose (map ->
// robot_frame) to the latest goal received on goal_pose (remapped to
// /target/predicted_pose in launch).

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "adibot_follower/a_star.hpp"

using namespace std::chrono_literals;

namespace
{
adibot_follower::Cell world_to_cell(
  double x, double y, const nav_msgs::msg::MapMetaData & info)
{
  return {
    static_cast<int>(std::floor((x - info.origin.position.x) / info.resolution)),
    static_cast<int>(std::floor((y - info.origin.position.y) / info.resolution))};
}

geometry_msgs::msg::PoseStamped cell_to_pose(
  const adibot_follower::Cell & c, const nav_msgs::msg::MapMetaData & info,
  const std_msgs::msg::Header & header)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header = header;
  pose.pose.position.x = info.origin.position.x + (c.col + 0.5) * info.resolution;
  pose.pose.position.y = info.origin.position.y + (c.row + 0.5) * info.resolution;
  pose.pose.orientation.w = 1.0;
  return pose;
}
}  // namespace

class GlobalPlanner : public rclcpp::Node
{
public:
  GlobalPlanner()
  : Node("global_planner"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    const double replan_rate = declare_parameter("replan_rate", 2.0);
    inflation_radius_ = declare_parameter("inflation_radius", 0.4);
    map_frame_ = declare_parameter("map_frame", "map");
    robot_frame_ = declare_parameter("robot_frame", "base_link");

    RCLCPP_INFO(
      get_logger(),
      "replan_rate %.1f Hz, inflation_radius %.2f m, map_frame '%s', robot_frame '%s'",
      replan_rate, inflation_radius_, map_frame_.c_str(), robot_frame_.c_str());

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", qos, [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {map_ = std::move(msg);});
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {goal_ = std::move(msg);});
    path_pub_ = create_publisher<nav_msgs::msg::Path>("plan", 10);

    timer_ = rclcpp::create_timer(
      this, get_clock(), rclcpp::Duration::from_seconds(1.0 / replan_rate), [this] {tick();});
  }

private:
  void tick()
  {
    if (!map_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for map");
      return;
    }
    if (!goal_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for goal");
      return;
    }

    geometry_msgs::msg::TransformStamped robot_tf;
    try {
      robot_tf = tf_buffer_.lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "tf unavailable: %s", ex.what());
      return;
    }

    adibot_follower::Grid grid;
    grid.width = static_cast<int>(map_->info.width);
    grid.height = static_cast<int>(map_->info.height);
    grid.occupied.resize(map_->data.size());
    for (size_t i = 0; i < map_->data.size(); ++i) {
      // unknown (-1) treated as occupied: conservative when the robot hasn't
      // fully observed the grid. Our maps are fully known (trinary, no -1
      // cells), so this only matters if a future map source leaves gaps.
      grid.occupied[i] = (map_->data[i] < 0 || map_->data[i] >= 50) ? 1 : 0;
    }

    const int inflation_cells =
      std::max(0, static_cast<int>(std::round(inflation_radius_ / map_->info.resolution)));
    const adibot_follower::Grid inflated = adibot_follower::inflate(grid, inflation_cells);

    const auto raw_start = world_to_cell(
      robot_tf.transform.translation.x, robot_tf.transform.translation.y, map_->info);
    const auto raw_goal = world_to_cell(
      goal_->pose.position.x, goal_->pose.position.y, map_->info);

    if (!inflated.in_bounds(raw_start.col, raw_start.row)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "robot is outside the map bounds");
      return;
    }
    // Snap the start the same way as the goal: the robot's true position can
    // end up inside the inflated margin (grazing an obstacle, localization
    // noise), and without this it can never plan an escape route -- A*
    // rejects an occupied start outright, so it would be stuck permanently
    // once it happens once. Verified empirically: the robot drove within
    // 0.62 m of a 0.42 m-radius obstacle and got permanently wedged, because
    // that put its own cell inside the inflated zone with no snap to fall
    // back on.
    const auto start = adibot_follower::find_nearest_free(inflated, raw_start);
    if (!start) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no free cell near the robot");
      return;
    }

    if (!inflated.in_bounds(raw_goal.col, raw_goal.row)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "goal is outside the map bounds");
      return;
    }
    const auto goal = adibot_follower::find_nearest_free(inflated, raw_goal);
    if (!goal) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no free cell near the goal");
      return;
    }

    const auto cells = adibot_follower::a_star(inflated, *start, *goal);
    if (cells.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no path found, keeping last plan");
      return;
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = now();
    path.poses.reserve(cells.size());
    for (const auto & c : cells) {
      path.poses.push_back(cell_to_pose(c, map_->info, path.header));
    }
    path_pub_->publish(path);
  }

  double inflation_radius_{};
  std::string map_frame_, robot_frame_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  geometry_msgs::msg::PoseStamped::SharedPtr goal_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GlobalPlanner>());
  rclcpp::shutdown();
  return 0;
}
