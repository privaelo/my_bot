// Copyright 2026 privaelo
//
// 8-connected A* on a row-major occupancy grid, plus grid inflation and
// nearest-free-cell snapping. Pure logic, no ROS types, so it's directly
// unit-testable on canned grids.

#ifndef ADIBOT_FOLLOWER__A_STAR_HPP_
#define ADIBOT_FOLLOWER__A_STAR_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

namespace adibot_follower
{

// Row-major occupancy grid in cell coordinates; occupied[row*width+col], 1 = blocked.
struct Grid
{
  int width{0};
  int height{0};
  std::vector<uint8_t> occupied;

  bool in_bounds(int col, int row) const
  {
    return col >= 0 && col < width && row >= 0 && row < height;
  }

  bool is_occupied(int col, int row) const
  {
    return occupied[static_cast<size_t>(row) * width + col] != 0;
  }
};

struct Cell
{
  int col{0};
  int row{0};

  bool operator==(const Cell & other) const {return col == other.col && row == other.row;}
};

inline Grid inflate(const Grid & grid, int radius_cells)
{
  Grid out = grid;
  if (radius_cells <= 0) {
    return out;
  }

  std::vector<std::pair<int, int>> disk;
  const int r2 = radius_cells * radius_cells;
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      if (dx * dx + dy * dy <= r2) {
        disk.emplace_back(dx, dy);
      }
    }
  }

  for (int row = 0; row < grid.height; ++row) {
    for (int col = 0; col < grid.width; ++col) {
      if (!grid.is_occupied(col, row)) {
        continue;
      }
      for (const auto & [dx, dy] : disk) {
        const int nc = col + dx;
        const int nr = row + dy;
        if (grid.in_bounds(nc, nr)) {
          out.occupied[static_cast<size_t>(nr) * grid.width + nc] = 1;
        }
      }
    }
  }
  return out;
}

// BFS ring search outward from `from` for the nearest free cell (returns `from`
// itself if already free). Used to snap an occupied goal cell to something A* can
// actually reach.
inline std::optional<Cell> find_nearest_free(const Grid & grid, Cell from)
{
  if (!grid.in_bounds(from.col, from.row)) {
    return std::nullopt;
  }
  if (!grid.is_occupied(from.col, from.row)) {
    return from;
  }

  static constexpr int kDCol[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static constexpr int kDRow[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  std::vector<uint8_t> visited(static_cast<size_t>(grid.width) * grid.height, 0);
  std::queue<Cell> q;
  q.push(from);
  visited[static_cast<size_t>(from.row) * grid.width + from.col] = 1;

  while (!q.empty()) {
    const Cell cur = q.front();
    q.pop();
    for (int i = 0; i < 8; ++i) {
      const Cell next{cur.col + kDCol[i], cur.row + kDRow[i]};
      if (!grid.in_bounds(next.col, next.row)) {
        continue;
      }
      const size_t idx = static_cast<size_t>(next.row) * grid.width + next.col;
      if (visited[idx]) {
        continue;
      }
      visited[idx] = 1;
      if (!grid.is_occupied(next.col, next.row)) {
        return next;
      }
      q.push(next);
    }
  }
  return std::nullopt;
}

// 8-connected A* with octile heuristic. Diagonal moves are rejected when both
// flanking orthogonal cells are occupied, so the path never cuts through the
// corner of two blocked cells. Returns an empty path if start/goal are invalid,
// occupied, or unreachable.
inline std::vector<Cell> a_star(const Grid & grid, Cell start, Cell goal)
{
  if (!grid.in_bounds(start.col, start.row) || !grid.in_bounds(goal.col, goal.row)) {
    return {};
  }
  if (grid.is_occupied(start.col, start.row) || grid.is_occupied(goal.col, goal.row)) {
    return {};
  }

  static constexpr int kDCol[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static constexpr int kDRow[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  static constexpr double kDiag = 1.4142135623730951;

  const size_t n = static_cast<size_t>(grid.width) * grid.height;
  auto index = [&](Cell c) {return static_cast<size_t>(c.row) * grid.width + c.col;};
  auto octile = [](Cell a, Cell b) {
      const double dx = std::abs(a.col - b.col);
      const double dy = std::abs(a.row - b.row);
      return (dx + dy) + (kDiag - 2.0) * std::min(dx, dy);
    };

  std::vector<double> g_cost(n, std::numeric_limits<double>::infinity());
  std::vector<int> came_from(n, -1);
  std::vector<uint8_t> closed(n, 0);

  using QueueEntry = std::pair<double, Cell>;
  auto cmp = [](const QueueEntry & a, const QueueEntry & b) {return a.first > b.first;};
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, decltype(cmp)> open(cmp);

  g_cost[index(start)] = 0.0;
  open.emplace(octile(start, goal), start);

  while (!open.empty()) {
    const Cell current = open.top().second;
    open.pop();
    const size_t ci = index(current);
    if (closed[ci]) {
      continue;
    }
    closed[ci] = 1;

    if (current == goal) {
      std::vector<Cell> path;
      int idx = static_cast<int>(ci);
      while (idx != -1) {
        path.push_back(Cell{idx % grid.width, idx / grid.width});
        idx = came_from[idx];
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    for (int i = 0; i < 8; ++i) {
      const int dcol = kDCol[i];
      const int drow = kDRow[i];
      const Cell next{current.col + dcol, current.row + drow};
      if (!grid.in_bounds(next.col, next.row) || grid.is_occupied(next.col, next.row)) {
        continue;
      }
      if (dcol != 0 && drow != 0 &&
        (grid.is_occupied(current.col + dcol, current.row) ||
        grid.is_occupied(current.col, current.row + drow)))
      {
        continue;
      }

      const size_t ni = index(next);
      if (closed[ni]) {
        continue;
      }
      const double step = (dcol != 0 && drow != 0) ? kDiag : 1.0;
      const double tentative = g_cost[ci] + step;
      if (tentative < g_cost[ni]) {
        g_cost[ni] = tentative;
        came_from[ni] = static_cast<int>(ci);
        open.emplace(tentative + octile(next, goal), next);
      }
    }
  }
  return {};
}

}  // namespace adibot_follower

#endif  // ADIBOT_FOLLOWER__A_STAR_HPP_
