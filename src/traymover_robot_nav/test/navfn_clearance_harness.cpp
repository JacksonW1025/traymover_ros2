// Offline fixture linked against the installed Nav2 NavFn implementation.
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include "nav2_navfn_planner/navfn.hpp"

int main(int argc, char ** argv)
{
  if (argc != 5) {
    return 2;
  }
  const std::string scenario(argv[1]);
  const double radius = std::stod(argv[2]);
  const double scaling = std::stod(argv[3]);
  const double inscribed = std::stod(argv[4]);
  constexpr int size = 120;
  constexpr double resolution = 0.05;
  const bool corner = scenario == "left" || scenario == "right";
  const bool mirror = scenario == "right";
  const int top = scenario == "narrow" ? 28 : 42;
  std::vector<unsigned char> costs(size * size, 0);
  std::vector<std::pair<int, int>> obstacles;
  for (int y = 0; y < size; ++y) {
    for (int x = 0; x < size; ++x) {
      const int logical_y = mirror ? size - 1 - y : y;
      const bool horizontal = x >= 10 && x <= 106 && logical_y >= 10 && logical_y <= top;
      const bool vertical = corner && x >= 74 && x <= 106 &&
        logical_y >= 10 && logical_y <= 110;
      if (!horizontal && !vertical) {
        obstacles.emplace_back(x, y);
        costs[y * size + x] = 254;
      }
    }
  }
  // Identical distance-to-cost formula to InflationLayer::computeCost. Taking
  // the maximum over all obstacles is equivalent to using the nearest one.
  const int cell_radius = static_cast<int>(std::ceil(radius / resolution));
  for (int y = 0; y < size; ++y) {
    for (int x = 0; x < size; ++x) {
      if (costs[y * size + x] == 254) {
        continue;
      }
      int distance_squared = size * size * 2;
      for (const auto & obstacle : obstacles) {
        const int dx = x - obstacle.first;
        const int dy = y - obstacle.second;
        distance_squared = std::min(distance_squared, dx * dx + dy * dy);
      }
      const double cells = std::sqrt(distance_squared);
      const double distance = cells * resolution;
      if (cells <= cell_radius) {
        costs[y * size + x] = distance <= inscribed ? 253 :
          static_cast<unsigned char>(252 * std::exp(-scaling * (distance - inscribed)));
      }
    }
  }
  int start[2] = {26, scenario == "narrow" ? 19 : (corner ? 26 : 20)};
  int goal[2] = {corner ? 90 : 96, corner ? 94 : start[1]};
  if (mirror) {
    start[1] = size - 1 - start[1];
    goal[1] = size - 1 - goal[1];
  }
  nav2_navfn_planner::NavFn planner(size, size);
  planner.setCostmap(costs.data(), true, false);
  // NavfnPlanner::makePlan computes the potential from robot to destination.
  planner.setStart(goal);
  planner.setGoal(start);
  planner.calcNavFnDijkstra(true);
  const int count = planner.calcPath(size * 4);
  if (count == 0) {
    std::cerr << "No path for " << scenario << '\n';
    return 1;
  }
  double clearance = 100;
  double center_error = 0;
  int middle_points = 0;
  for (int i = 0; i < count; ++i) {
    const double x = planner.getPathX()[i];
    const double y = planner.getPathY()[i];
    // Clearance is to occupied cell edges, not to their centers.
    for (const auto & obstacle : obstacles) {
      const double dx = std::max(0.0, std::abs(x - obstacle.first) - 0.5);
      const double dy = std::max(0.0, std::abs(y - obstacle.second) - 0.5);
      clearance = std::min(clearance, std::hypot(dx, dy) * resolution);
    }
    if (!corner && x >= 45 && x <= 77) {
      center_error += std::abs(y - (10 + top) / 2.0) * resolution;
      ++middle_points;
    }
  }
  std::cout << count << ' ' << clearance << ' ' <<
    (middle_points ? center_error / middle_points : 0) << '\n';
}
