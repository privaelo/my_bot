// Copyright 2026 privaelo

#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "adibot_follower/a_star.hpp"

namespace
{
using adibot_follower::Cell;
using adibot_follower::Grid;

Grid make_grid(int width, int height)
{
  Grid grid;
  grid.width = width;
  grid.height = height;
  grid.occupied.assign(static_cast<size_t>(width) * height, 0);
  return grid;
}

void block(Grid & grid, int col, int row)
{
  grid.occupied[static_cast<size_t>(row) * grid.width + col] = 1;
}

bool crosses_occupied(const Grid & grid, const std::vector<Cell> & path)
{
  for (const auto & c : path) {
    if (grid.is_occupied(c.col, c.row)) {
      return true;
    }
  }
  return false;
}
}  // namespace

TEST(AStar, OpenGridFindsDirectPath)
{
  const Grid grid = make_grid(5, 5);
  const auto path = adibot_follower::a_star(grid, {0, 0}, {4, 4});
  ASSERT_FALSE(path.empty());
  EXPECT_EQ(path.front(), (Cell{0, 0}));
  EXPECT_EQ(path.back(), (Cell{4, 4}));
  EXPECT_FALSE(crosses_occupied(grid, path));
}

TEST(AStar, WallWithGapForcesDetour)
{
  // Solid wall down column 2 except a gap at row 4.
  Grid grid = make_grid(5, 9);
  for (int row = 0; row < grid.height; ++row) {
    if (row != 4) {
      block(grid, 2, row);
    }
  }
  const auto path = adibot_follower::a_star(grid, {0, 0}, {4, 0});
  ASSERT_FALSE(path.empty());
  EXPECT_FALSE(crosses_occupied(grid, path));
  const bool routes_through_gap =
    std::find(path.begin(), path.end(), Cell{2, 4}) != path.end();
  EXPECT_TRUE(routes_through_gap);
}

TEST(AStar, OccupiedGoalSnapsToNearestFreeCell)
{
  Grid grid = make_grid(5, 5);
  block(grid, 2, 2);
  const auto snapped = adibot_follower::find_nearest_free(grid, {2, 2});
  ASSERT_TRUE(snapped.has_value());
  EXPECT_FALSE(grid.is_occupied(snapped->col, snapped->row));

  const auto path = adibot_follower::a_star(grid, {0, 0}, *snapped);
  EXPECT_FALSE(path.empty());
}

TEST(AStar, GoalEnclosedByOccupiedRingIsUnreachable)
{
  Grid grid = make_grid(5, 5);
  // Ring of occupied cells around the free center (2,2) seals it off entirely,
  // including diagonal neighbors so corner-cut prevention can't sneak through.
  for (int drow = -1; drow <= 1; ++drow) {
    for (int dcol = -1; dcol <= 1; ++dcol) {
      if (dcol != 0 || drow != 0) {
        block(grid, 2 + dcol, 2 + drow);
      }
    }
  }
  const auto path = adibot_follower::a_star(grid, {0, 0}, {2, 2});
  EXPECT_TRUE(path.empty());
}
