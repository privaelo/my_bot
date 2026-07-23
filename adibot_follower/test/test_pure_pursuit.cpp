// Copyright 2026 privaelo

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "adibot_follower/pure_pursuit.hpp"

namespace
{
using adibot_follower::Point2D;
using adibot_follower::Pose2D;

std::vector<Point2D> straight_path_along_x(double from, double to, double step)
{
  std::vector<Point2D> path;
  for (double x = from; x <= to + 1e-9; x += step) {
    path.push_back({x, 0.0});
  }
  return path;
}
}  // namespace

TEST(PurePursuit, LookaheadFromPathStart)
{
  const auto path = straight_path_along_x(0.0, 10.0, 1.0);
  const auto point = adibot_follower::find_lookahead_point(path, {0.0, 0.0, 0.0}, 3.0);
  ASSERT_TRUE(point.has_value());
  EXPECT_NEAR(point->x, 3.0, 1e-6);
  EXPECT_NEAR(point->y, 0.0, 1e-6);
}

TEST(PurePursuit, LookaheadWalksForwardFromNearestPoint)
{
  const auto path = straight_path_along_x(0.0, 10.0, 1.0);
  const auto point = adibot_follower::find_lookahead_point(path, {5.0, 0.0, 0.0}, 3.0);
  ASSERT_TRUE(point.has_value());
  EXPECT_NEAR(point->x, 8.0, 1e-6);
}

TEST(PurePursuit, LookaheadClampsToLastPointWhenPathIsShort)
{
  const std::vector<Point2D> path{{0.0, 0.0}, {1.0, 0.0}};
  const auto point = adibot_follower::find_lookahead_point(path, {0.0, 0.0, 0.0}, 5.0);
  ASSERT_TRUE(point.has_value());
  EXPECT_NEAR(point->x, 1.0, 1e-6);
  EXPECT_NEAR(point->y, 0.0, 1e-6);
}

TEST(PurePursuit, LookaheadOnEmptyPathReturnsNullopt)
{
  const std::vector<Point2D> path;
  EXPECT_FALSE(adibot_follower::find_lookahead_point(path, {0.0, 0.0, 0.0}, 3.0).has_value());
}

TEST(PurePursuit, CurvatureToPointAheadIsZero)
{
  const double curvature = adibot_follower::curvature_to_point({0.0, 0.0, 0.0}, {5.0, 0.0});
  EXPECT_NEAR(curvature, 0.0, 1e-6);
}

TEST(PurePursuit, CurvatureToPointAbeamLeftIsPositive)
{
  // local_x=0, local_y=L=2 -> curvature = 2*L/L^2 = 2/L = 1.0
  const double curvature = adibot_follower::curvature_to_point({0.0, 0.0, 0.0}, {0.0, 2.0});
  EXPECT_NEAR(curvature, 1.0, 1e-6);
}

TEST(PurePursuit, CurvatureToPointAbeamRightIsNegative)
{
  const double curvature = adibot_follower::curvature_to_point({0.0, 0.0, 0.0}, {0.0, -2.0});
  EXPECT_NEAR(curvature, -1.0, 1e-6);
}

TEST(PurePursuit, CurvatureAccountsForRobotYaw)
{
  // Robot facing +y (yaw = pi/2); target due north of it is still "ahead".
  const double curvature = adibot_follower::curvature_to_point(
    {0.0, 0.0, M_PI_2}, {0.0, 5.0});
  EXPECT_NEAR(curvature, 0.0, 1e-6);
}
