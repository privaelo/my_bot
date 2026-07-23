// Copyright 2026 privaelo
//
// Pure-pursuit geometry: lookahead point selection and curvature-to-point,
// on plain 2D points/poses. No ROS types, so it's directly unit-testable on
// canned paths.

#ifndef ADIBOT_FOLLOWER__PURE_PURSUIT_HPP_
#define ADIBOT_FOLLOWER__PURE_PURSUIT_HPP_

#include <cmath>
#include <optional>
#include <vector>

namespace adibot_follower
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

// Robot pose in the same frame as the path (map frame): position + heading.
struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// Walks `path` from the point nearest `robot`, returning the first point at
// or past `lookahead_distance`. Clamps to the last point if the remaining
// path is shorter than the lookahead. Returns nullopt only for an empty path.
inline std::optional<Point2D> find_lookahead_point(
  const std::vector<Point2D> & path, const Pose2D & robot, double lookahead_distance)
{
  if (path.empty()) {
    return std::nullopt;
  }

  auto dist = [&robot](const Point2D & p) {
      return std::hypot(p.x - robot.x, p.y - robot.y);
    };

  size_t nearest = 0;
  double nearest_dist = dist(path[0]);
  for (size_t i = 1; i < path.size(); ++i) {
    const double d = dist(path[i]);
    if (d < nearest_dist) {
      nearest_dist = d;
      nearest = i;
    }
  }

  for (size_t i = nearest; i < path.size(); ++i) {
    if (dist(path[i]) >= lookahead_distance) {
      return path[i];
    }
  }
  return path.back();
}

// Pure-pursuit curvature (1/m) to steer from `robot` toward `target`, using
// REP-103 body-frame convention (x-forward, y-left): curvature = 2*y/L^2.
// Positive curvature turns left, matching angular.z's sign directly.
inline double curvature_to_point(const Pose2D & robot, const Point2D & target)
{
  const double dx = target.x - robot.x;
  const double dy = target.y - robot.y;
  const double local_x = dx * std::cos(robot.yaw) + dy * std::sin(robot.yaw);
  const double local_y = -dx * std::sin(robot.yaw) + dy * std::cos(robot.yaw);
  const double l2 = local_x * local_x + local_y * local_y;
  if (l2 < 1e-9) {
    return 0.0;
  }
  return 2.0 * local_y / l2;
}

}  // namespace adibot_follower

#endif  // ADIBOT_FOLLOWER__PURE_PURSUIT_HPP_
