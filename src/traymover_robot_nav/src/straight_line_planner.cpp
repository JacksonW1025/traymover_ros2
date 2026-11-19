#include <algorithm>
#include <cmath>
#include <cstddef>
#include <mutex>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_navfn_planner/navfn_planner.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace traymover_robot_nav
{
namespace
{
using nav2_costmap_2d::Costmap2D;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::PoseStamped;
using Poses = std::vector<PoseStamped>;

// Allow small cost quantization differences between NavFn samples and the chord.
constexpr int kInflationCostTolerance = 3;

struct Cell
{
  unsigned int x{};
  unsigned int y{};
};

Point interpolate(const Point & from, const Point & to, double fraction)
{
  Point point;
  point.x = from.x + fraction * (to.x - from.x);
  point.y = from.y + fraction * (to.y - from.y);
  point.z = from.z + fraction * (to.z - from.z);
  return point;
}

int pointCost(const Costmap2D & map, const Point & point)
{
  Cell cell;
  if (!map.worldToMap(point.x, point.y, cell.x, cell.y)) {
    return nav2_costmap_2d::NO_INFORMATION;
  }
  return map.getCost(cell.x, cell.y);
}

bool canStraighten(
  const Costmap2D & map, const Point & from, const Point & to, std::size_t steps,
  const std::vector<int> & costs, std::size_t first, std::size_t last)
{
  double original_sum = 0.0;
  int original_peak = 0;
  for (std::size_t i = first; i <= last; ++i) {
    if (costs[i] >= INSCRIBED_INFLATED_OBSTACLE) {
      return false;
    }
    original_sum += costs[i];
    original_peak = std::max(original_peak, costs[i]);
  }
  const int max_cost = std::min(
    original_peak + kInflationCostTolerance, int{INSCRIBED_INFLATED_OBSTACLE} - 1);
  double shortcut_sum = 0.0;
  Cell previous;
  for (std::size_t i = 0; i <= steps; ++i) {
    const auto point = interpolate(from, to, static_cast<double>(i) / steps);
    Cell current;
    if (!map.worldToMap(point.x, point.y, current.x, current.y)) {
      return false;
    }
    const int cost = map.getCost(current.x, current.y);
    if (cost > max_cost) {
      return false;
    }
    // Samples are at most half a cell apart. A diagonal transition can
    // cross either side cell, so both must be known and traversable.
    if (i > 0 && current.x != previous.x && current.y != previous.y &&
      (map.getCost(previous.x, current.y) >= INSCRIBED_INFLATED_OBSTACLE ||
      map.getCost(current.x, previous.y) >= INSCRIBED_INFLATED_OBSTACLE))
    {
      return false;
    }
    shortcut_sum += cost;
    previous = current;
  }
  return shortcut_sum / (steps + 1) <=
         original_sum / (last - first + 1) + kInflationCostTolerance;
}

std::size_t splitAtBend(const Poses & poses, std::size_t first, std::size_t last)
{
  const auto & from = poses[first].pose.position;
  const auto & to = poses[last].pose.position;
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const std::size_t span = last - first;
  std::size_t split = first + span / 2;
  double largest_deviation = 0.0;
  // Keep subdivisions balanced even if the largest bend is near an endpoint.
  const std::size_t margin = std::max<std::size_t>(1, span / 4);
  for (std::size_t i = first + margin; i <= last - margin; ++i) {
    const auto & point = poses[i].pose.position;
    const double deviation = std::abs(dx * (point.y - from.y) - dy * (point.x - from.x));
    if (deviation > largest_deviation) {
      largest_deviation = deviation;
      split = i;
    }
  }
  return split;
}

void appendStraightened(
  const Costmap2D & map, const Poses & poses, const std::vector<int> & costs,
  std::size_t first, std::size_t last, Poses & result)
{
  if (last == first + 1) {
    result.push_back(poses[last]);
    return;
  }

  const auto & from = poses[first].pose.position;
  const auto & to = poses[last].pose.position;
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double distance = std::hypot(dx, dy);
  if (distance > 0.0) {
    const auto steps = static_cast<std::size_t>(std::ceil(distance / (0.5 * map.getResolution())));
    if (canStraighten(map, from, to, steps, costs, first, last)) {
      PoseStamped point = poses[first];
      const double half_yaw = 0.5 * std::atan2(dy, dx);
      point.pose.orientation.x = 0.0;
      point.pose.orientation.y = 0.0;
      point.pose.orientation.z = std::sin(half_yaw);
      point.pose.orientation.w = std::cos(half_yaw);
      // RPP still needs dense path samples for its lookahead and pruning.
      for (std::size_t i = 1; i < steps; ++i) {
        point.pose.position = interpolate(from, to, static_cast<double>(i) / steps);
        result.push_back(point);
      }
      result.push_back(poses[last]);
      return;
    }
  }

  const auto split = splitAtBend(poses, first, last);
  appendStraightened(map, poses, costs, first, split, result);
  appendStraightened(map, poses, costs, split, last, result);
}
}  // namespace

class StraightLinePlanner : public nav2_navfn_planner::NavfnPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    map_ = costmap_ros->getCostmap();
    NavfnPlanner::configure(parent, std::move(name), std::move(tf), std::move(costmap_ros));
  }

  nav_msgs::msg::Path createPlan(const PoseStamped & start, const PoseStamped & goal) override
  {
    auto path = NavfnPlanner::createPlan(start, goal);
    if (path.poses.size() < 3) {
      return path;
    }

    std::lock_guard<Costmap2D::mutex_t> lock(*map_->getMutex());
    std::vector<int> costs;
    costs.reserve(path.poses.size());
    for (const auto & pose : path.poses) {
      costs.push_back(pointCost(*map_, pose.pose.position));
    }

    Poses result;
    result.reserve(path.poses.size());
    result.push_back(path.poses.front());
    appendStraightened(*map_, path.poses, costs, 0, path.poses.size() - 1, result);
    path.poses = std::move(result);
    return path;
  }

private:
  Costmap2D * map_{nullptr};
};
}  // namespace traymover_robot_nav

PLUGINLIB_EXPORT_CLASS(traymover_robot_nav::StraightLinePlanner, nav2_core::GlobalPlanner)
