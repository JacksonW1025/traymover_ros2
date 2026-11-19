#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "nav2_core/exceptions.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "pluginlib/class_loader.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace traymover_robot_nav
{
namespace
{
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Path;
using nav2_costmap_2d::Costmap2D;
using SteadyClock = std::chrono::steady_clock;
constexpr double kStep = 0.035;
constexpr double kMaxHeadingChange = 0.90;
constexpr double kHalfLength = 0.40;
constexpr double kHalfWidth = 0.275;
constexpr double kSensorFront = 0.50;
constexpr double kSensorWidth = 0.11;
constexpr double kObstacleAssociationDistance = 0.25;

class HoldPosition : public nav2_core::PlannerException
{
public:
  using PlannerException::PlannerException;
};

struct Point
{
  double x;
  double y;

  Point operator+(Point other) const {return {x + other.x, y + other.y};}
  Point operator-(Point other) const {return {x - other.x, y - other.y};}
  Point operator*(double scale) const {return {x * scale, y * scale};}
  double norm() const {return std::hypot(x, y);}
};

struct ObstacleFrame
{
  std::vector<Point> points;
  std::vector<Point> triggers;
};

Point xy(const PoseStamped & pose)
{
  return {pose.pose.position.x, pose.pose.position.y};
}

PoseStamped poseAt(const std_msgs::msg::Header & header, Point p, double yaw)
{
  PoseStamped pose;
  pose.header = header;
  pose.pose.position.x = p.x;
  pose.pose.position.y = p.y;
  pose.pose.orientation.z = std::sin(0.5 * yaw);
  pose.pose.orientation.w = std::cos(0.5 * yaw);
  return pose;
}

double heading(Point direction)
{
  return std::atan2(direction.y, direction.x);
}

bool aligned(double a, double b)
{
  return std::abs(std::remainder(a - b, 2 * M_PI)) <= kMaxHeadingChange;
}

double smooth(double u)
{
  u = std::min(1.0, std::max(0.0, u));
  return u * u * (3 - 2 * u);
}

struct Route
{
  Path path;
  std::vector<double> distance;

  bool empty() const {return path.poses.empty();}

  Point at(double s) const
  {
    const auto upper = std::upper_bound(distance.begin(), distance.end(), s);
    if (upper == distance.end()) {
      return xy(path.poses.back());
    }
    if (upper == distance.begin()) {
      return xy(path.poses.front());
    }
    const auto i = static_cast<std::size_t>(upper - distance.begin());
    const auto a = xy(path.poses[i - 1]), b = xy(path.poses[i]);
    return a + (b - a) * ((s - distance[i - 1]) / (distance[i] - distance[i - 1]));
  }

  double yaw(double s) const
  {
    return heading(at(s + 0.08) - at(s - 0.08));
  }

  void append(PoseStamped pose)
  {
    const double length = empty() ? 0.0 : (xy(pose) - xy(path.poses.back())).norm();
    if (empty() || length > 1e-6) {
      distance.push_back(empty() ? 0.0 : distance.back() + length);
      path.poses.push_back(std::move(pose));
    } else {
      path.poses.back() = std::move(pose);
    }
  }

  double project(Point p, double previous) const
  {
    double best = std::numeric_limits<double>::infinity(), along = previous;
    for (double s = previous; s <= std::min(distance.back(), previous + 1.0); s += kStep) {
      const double error = (p - at(s)).norm();
      if (error < best) {
        best = error;
        along = s;
      }
    }
    if (distance.back() - along < kStep && (p - at(distance.back())).norm() < best) {
      return distance.back();
    }
    return along;
  }
};

bool insideFootprint(Point p, double padding)
{
  return (std::abs(p.x) <= kHalfLength + padding && std::abs(p.y) <= kHalfWidth + padding) ||
         (p.x >= kHalfLength - padding && p.x <= kSensorFront + padding &&
         std::abs(p.y) <= kSensorWidth + padding);
}

Point relative(Point delta, double c, double s)
{
  return {c * delta.x + s * delta.y, c * delta.y - s * delta.x};
}

bool knownFootprint(const Costmap2D & map, Point p, double yaw, double clearance = 0.0)
{
  unsigned int mx, my;
  if (!map.worldToMap(p.x, p.y, mx, my)) {
    return false;
  }
  const double resolution = map.getResolution();
  const double padding = 0.72 * resolution + clearance;
  const double c = std::cos(yaw), s = std::sin(yaw);
  const int radius = static_cast<int>(std::ceil((0.52 + 1.42 * padding) / resolution)) + 1;
  for (int y = static_cast<int>(my) - radius; y <= static_cast<int>(my) + radius; ++y) {
    for (int x = static_cast<int>(mx) - radius; x <= static_cast<int>(mx) + radius; ++x) {
      const Point center{map.getOriginX() + (x + 0.5) * resolution,
        map.getOriginY() + (y + 0.5) * resolution};
      if (!insideFootprint(relative(center - p, c, s), padding)) {
        continue;
      }
      if (x < 0 || y < 0 || x >= static_cast<int>(map.getSizeInCellsX()) ||
        y >= static_cast<int>(map.getSizeInCellsY()) ||
        map.getCost(x, y) >= nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        return false;
      }
    }
  }
  return true;
}

bool obstacleAt(const std::vector<Point> & points, Point p, double yaw, double clearance)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  return std::any_of(points.begin(), points.end(), [&](Point obstacle) {
    const Point delta = obstacle - p;
    const double radius = 0.52 + 1.42 * clearance;
    return std::abs(delta.x) <= radius && std::abs(delta.y) <= radius &&
           insideFootprint(relative(delta, c, s), clearance);
  });
}

bool clearRoute(
  const Costmap2D & map, const Route & route, double from, double to,
  const std::vector<Point> & obstacles = {}, double clearance = 0.0,
  double wall_clearance = 0.0)
{
  const auto clear = [&](double s) {
      const auto point = route.at(s);
      const double yaw = route.distance.back() > kStep ? route.yaw(s) :
        tf2::getYaw(route.path.poses.back().pose.orientation);
      return !obstacleAt(obstacles, point, yaw, clearance) &&
             knownFootprint(map, point, yaw, wall_clearance);
    };
  const double step = std::min(kStep, map.getResolution() * 0.5);
  for (double s = from; s < to; s += step) {
    if (!clear(s)) {
      return false;
    }
  }
  return clear(to);
}

bool clearEntryTurn(
  const Costmap2D & map, const PoseStamped & start, double target,
  const std::vector<Point> & obstacles, double clearance, double wall_clearance)
{
  const double initial = tf2::getYaw(start.pose.orientation);
  const double change = std::remainder(target - initial, 2 * M_PI);
  const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(change) / 0.04)));
  for (int i = 0; i <= steps; ++i) {
    const double yaw = initial + change * i / steps;
    if (!knownFootprint(map, xy(start), yaw, wall_clearance) ||
      obstacleAt(obstacles, xy(start), yaw, clearance))
    {
      return false;
    }
  }
  return true;
}

Path remaining(const Route & route, double progress, const builtin_interfaces::msg::Time & stamp)
{
  Path path;
  path.header = route.path.header;
  path.header.stamp = stamp;
  const auto first = std::upper_bound(route.distance.begin(), route.distance.end(), progress);
  if (first != route.distance.end()) {
    path.poses.push_back(poseAt(path.header, route.at(progress), route.yaw(progress)));
  }
  path.poses.insert(path.poses.end(),
    route.path.poses.begin() + (first - route.distance.begin()), route.path.poses.end());
  if (path.poses.empty()) {
    path.poses.push_back(route.path.poses.back());
  }
  for (auto & pose : path.poses) {
    pose.header = path.header;
  }
  return path;
}

Route makePatch(
  const Route & route, const PoseStamped & start, double from, double to,
  double rise, double fall, double shift, double entry_slope)
{
  Route patch;
  patch.path.header = route.path.header;
  patch.append(start);
  const double start_yaw = tf2::getYaw(start.pose.orientation);
  const Point error = xy(start) - route.at(from);
  const auto steps = static_cast<int>(std::ceil((to - from) / kStep));
  for (int i = 1; i <= steps; ++i) {
    const double offset = (to - from) * i / steps;
    const double s = from + offset;
    const double yaw = route.yaw(s);
    const Point normal{-std::sin(yaw), std::cos(yaw)};
    const double u = std::min(offset / rise, 1.0);
    const double entry = smooth(u) + entry_slope * u * (1 - u) * (1 - u);
    const double lateral = shift * entry * smooth((to - s) / fall);
    const Point p = route.at(s) + normal * lateral + error * (1 - smooth(offset / rise));
    const double travel_yaw = heading(p - xy(patch.path.poses.back()));
    if (!aligned(yaw, travel_yaw) || (i == 1 && !aligned(start_yaw, travel_yaw))) {
      return Route();
    }
    patch.append(poseAt(patch.path.header, p, travel_yaw));
  }
  return patch;
}
}  // namespace

class Option22LocalDetour : public nav2_core::GlobalPlanner
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override
  {
    node_ = parent;
    tf_ = tf;
    map_ = costmap->getCostmap();
    baseline_ = loader_.createSharedInstance("traymover_robot_nav::StraightLinePlanner");
    baseline_->configure(parent, name, std::move(tf), std::move(costmap));
    auto node = parent.lock();
    const auto parameter = [&](const std::string & key, double fallback) {
        const auto qualified = name + "." + key;
        return node->has_parameter(qualified) ?
               node->get_parameter(qualified).as_double() :
               node->declare_parameter(qualified, fallback);
      };
    wait_ = parameter("wait_before_detour_sec", 0.0);
    max_lateral_ = parameter("detour_max_lateral_m", 0.75);
    max_length_ = parameter("detour_max_length_m", 3.80);
    clearance_ = parameter("detour_clearance_m", 0.14);
    wall_clearance_ = parameter("detour_wall_clearance_m", 0.06);
    tracking_clearance_ = parameter("tracking_clearance_m", 0.08);
    if (!(std::isfinite(wait_) && wait_ >= 0 &&
      max_lateral_ >= 0.3 && max_lateral_ <= 1.5 &&
      std::isfinite(max_length_) && max_length_ > 0 && tracking_clearance_ > 0 &&
      clearance_ >= tracking_clearance_ && clearance_ <= 0.30 &&
      wall_clearance_ >= 0.0 && wall_clearance_ <= 0.30))
    {
      throw nav2_core::PlannerException("Invalid option 22 detour parameters");
    }

    state_ = node->create_publisher<std_msgs::msg::String>("/option22/planner_state", 1);
    entry_ = node->create_publisher<PoseStamped>("/option22/detour_entry", 1);
    camera_stop_ = node->create_subscription<std_msgs::msg::Bool>(
      "/option22/auto_request", 1,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {camera_stop_requested_ = msg->data;});
    obstacles_ = node->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/option22/obstacles", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        cloud_ = cloud;
        cloud_received_ = SteadyClock::now();
      });
  }

  void cleanup() override
  {
    obstacles_.reset();
    camera_stop_.reset();
    camera_stop_requested_ = false;
    state_.reset();
    entry_.reset();
    baseline_->cleanup();
    baseline_.reset();
    route_ = Route();
    detour_ = Route();
    remembered_obstacles_.clear();
    replacement_pending_ = false;
    waiting_ = false;
  }

  void activate() override
  {
    baseline_->activate();
    state_->on_activate();
    entry_->on_activate();
  }

  void deactivate() override
  {
    state_->on_deactivate();
    entry_->on_deactivate();
    baseline_->deactivate();
  }

  Path createPlan(const PoseStamped & start, const PoseStamped & goal) override
  {
    std::lock_guard<std::mutex> lock(plan_mutex_);
    const auto robot = tf_->lookupTransform("map", "base_link", tf2::TimePointZero);
    const Point robot_xy{robot.transform.translation.x, robot.transform.translation.y};
    const bool current_leg = (xy(start) - robot_xy).norm() < 0.30;
    try {
      return current_leg ? planCurrentLeg(start, goal) : staticRoute(start, goal).path;
    } catch (const HoldPosition &) {
      throw;
    } catch (const std::exception & error) {
      if (current_leg) {
        publish("ROUTE_UNAVAILABLE");
      }
      throw nav2_core::PlannerException(error.what());
    }
  }

private:
  Route staticRoute(const PoseStamped & start, const PoseStamped & goal)
  {
    const auto plan = baseline_->createPlan(start, goal);
    Route route;
    route.path.header = plan.header;
    route.append(start);
    for (const auto & pose : plan.poses) {
      route.append(pose);
    }
    if (plan.poses.empty() || (xy(plan.poses.back()) - xy(goal)).norm() > kStep) {
      throw std::runtime_error("No static route to the exact goal");
    }
    route.append(goal);
    std::lock_guard<Costmap2D::mutex_t> lock(*map_->getMutex());
    if (!clearRoute(*map_, route, 0.0, route.distance.back()) ||
      !knownFootprint(*map_, xy(goal), tf2::getYaw(goal.pose.orientation)))
    {
      throw std::runtime_error("Static route footprint crosses unknown or occupied space");
    }
    return route;
  }

  void publish(const std::string & state)
  {
    std_msgs::msg::String msg;
    msg.data = state;
    state_->publish(msg);
  }

  Path hold(const std::string & state)
  {
    publish(state);
    throw HoldPosition(state);
  }

  ObstacleFrame latestObstacles()
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      if (SteadyClock::now() - cloud_received_ > std::chrono::duration<double>(0.45)) {
        throw std::runtime_error("Option 22 obstacle snapshot is stale");
      }
      cloud = cloud_;
    }
    auto node = node_.lock();
    if (!cloud || cloud->header.frame_id != "map" ||
      std::abs((node->now() - rclcpp::Time(cloud->header.stamp, node->get_clock()->get_clock_type())).seconds()) > 0.45)
    {
      throw std::runtime_error("Option 22 obstacle snapshot is unavailable");
    }
    ObstacleFrame frame;
    if (cloud->width > 0) {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x"), y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> trigger(*cloud, "detour_trigger");
      for (; x != x.end(); ++x, ++y, ++trigger) {
        if (std::isfinite(*x) && std::isfinite(*y)) {
          frame.points.push_back({*x, *y});
          if (*trigger > 0.5f) {
            frame.triggers.push_back({*x, *y});
          }
        }
      }
    }
    return frame;
  }

  void rememberTrigger(const std::vector<Point> & points, const std::vector<Point> & triggers)
  {
    // Keep the observed surface that caused a detour, even after the sensor turns away.
    for (Point point : points) {
      const bool near_trigger = std::any_of(
        triggers.begin(), triggers.end(),
        [&](Point trigger) {return (trigger - point).norm() <= 0.22;});
      if (near_trigger && std::none_of(
          remembered_obstacles_.begin(), remembered_obstacles_.end(),
          [&](Point old) {return (old - point).norm() < kStep;}))
      {
        remembered_obstacles_.push_back(point);
      }
    }
  }

  Path planCurrentLeg(const PoseStamped & start, const PoseStamped & goal)
  {
    if (camera_stop_requested_) {
      return hold("CAMERA_ESTOP");
    }
    const auto frame = latestObstacles();
    const auto now = SteadyClock::now();
    const bool resumed = detour_.empty() && now - last_request_ > std::chrono::seconds(3);
    last_request_ = now;
    if (resumed || route_.empty() || (xy(goal) - xy(route_.path.poses.back())).norm() > 1e-4 ||
      std::abs(std::remainder(tf2::getYaw(goal.pose.orientation) -
      tf2::getYaw(route_.path.poses.back().pose.orientation), 2 * M_PI)) > 1e-4)
    {
      route_ = Route();
      detour_ = Route();
      remembered_obstacles_.clear();
      replacement_pending_ = false;
      waiting_ = false;
      progress_ = 0.0;
      route_ = staticRoute(start, goal);
    }
    const builtin_interfaces::msg::Time stamp = node_.lock()->now();
    std::lock_guard<Costmap2D::mutex_t> lock(*map_->getMutex());
    progress_ = route_.project(xy(start), progress_);
    if (!detour_.empty()) {
      detour_progress_ = detour_.project(xy(start), detour_progress_);
      const double end = detour_.distance.back();
      if (end - detour_progress_ <= 0.12 && (xy(start) - detour_.at(end)).norm() <= 0.16) {
        detour_ = Route();
        remembered_obstacles_.clear();
        replacement_pending_ = false;
        progress_ = std::max(progress_, rejoin_);
      } else {
        return continueDetour(start, frame, stamp);
      }
    }

    if ((xy(start) - route_.at(progress_)).norm() > 0.65) {
      return hold("OFF_ROUTE");
    }
    const double horizon = std::min(progress_ + max_length_, route_.distance.back());
    bool blocked = false;
    double last = progress_;
    if (!frame.triggers.empty()) {
      for (double s = progress_; ; s = std::min(s + kStep, horizon)) {
        if (obstacleAt(blocked ? frame.points : frame.triggers, route_.at(s), route_.yaw(s),
          tracking_clearance_))
        {
          blocked = true;
          last = s;
        }
        if (s == horizon || (blocked && s - last > 0.18)) {
          break;
        }
      }
    }

    if (!blocked) {
      if (!clearRoute(*map_, route_, progress_, horizon)) {
        return hold("ROUTE_UNAVAILABLE");
      }
      waiting_ = false;
      publish("FOLLOWING_ROUTE");
      return remaining(route_, progress_, stamp);
    }

    if (!waiting_) {
      blocked_since_ = now;
      waiting_ = true;
    }
    if (now - blocked_since_ < std::chrono::duration<double>(wait_)) {
      return hold("WAITING");
    }
    auto candidate = selectDetour(start, frame.points, last, progress_, horizon, stamp);
    if (candidate.poses.empty()) {
      return hold("NO_LOCAL_DETOUR");
    }
    rememberTrigger(frame.points, frame.triggers);
    return candidate;
  }

  Path continueDetour(
    const PoseStamped & start, const ObstacleFrame & frame,
    const builtin_interfaces::msg::Time & stamp)
  {
    const double end = detour_.distance.back();
    const double yaw = tf2::getYaw(start.pose.orientation);
    const double heading_error = yaw - detour_.yaw(detour_progress_);
    const bool at_entry = detour_progress_ <= kStep;
    const double entry_yaw = tf2::getYaw(entry_pose_.pose.orientation);
    if ((xy(start) - detour_.at(detour_progress_)).norm() > 0.45 ||
      std::cos(heading_error) < 0.0 ||
      !clearRoute(*map_, detour_, detour_progress_, end) ||
      (at_entry && !clearEntryTurn(*map_, start, entry_yaw, {}, 0.0, 0.0)))
    {
      // A temporary tracking or map fault can stop motion without changing the route.
      return hold("DETOUR_BLOCKED");
    }

    std::vector<Point> new_triggers;
    for (Point trigger : frame.triggers) {
      const bool already_bypassing = std::any_of(
        remembered_obstacles_.begin(), remembered_obstacles_.end(),
        [&](Point point) {return (point - trigger).norm() <= kObstacleAssociationDistance;});
      if (!already_bypassing) {
        new_triggers.push_back(trigger);
      }
    }
    if (!new_triggers.empty() &&
      (!clearRoute(*map_, detour_, detour_progress_, end, new_triggers, tracking_clearance_) ||
      (at_entry && !clearEntryTurn(
        *map_, start, entry_yaw, new_triggers, tracking_clearance_, 0.0))))
    {
      rememberTrigger(frame.points, new_triggers);
      replacement_pending_ = true;
    }
    if (!replacement_pending_) {
      return activeDetour(stamp);
    }

    auto obstacles = frame.points;
    obstacles.insert(obstacles.end(), remembered_obstacles_.begin(), remembered_obstacles_.end());
    const double horizon = std::min(progress_ + max_length_, route_.distance.back());
    double last = progress_;
    for (double s = progress_; s <= horizon; s += kStep) {
      if (obstacleAt(obstacles, route_.at(s), route_.yaw(s), tracking_clearance_)) {
        last = s;
      }
    }
    auto candidate = selectDetour(start, obstacles, last, rejoin_, horizon, stamp);
    if (candidate.poses.empty()) {
      return hold("DETOUR_BLOCKED");
    }
    replacement_pending_ = false;
    return candidate;
  }

  Path selectDetour(
    const PoseStamped & start, const std::vector<Point> & obstacles, double last,
    double earliest_rejoin, double horizon, const builtin_interfaces::msg::Time & stamp)
  {
    auto path = searchDetour(start, obstacles, last, earliest_rejoin, horizon, stamp,
        clearance_, wall_clearance_);
    if (path.poses.empty()) {
      // A narrow corridor can fit the footprint with less planning padding.
      // Retain the tracking clearance and the known-map footprint checks.
      return searchDetour(start, obstacles, last, earliest_rejoin, horizon, stamp,
          std::min(clearance_, tracking_clearance_ + 0.03), std::min(wall_clearance_, 0.02));
    }
    return path;
  }

  Path searchDetour(
    const PoseStamped & start, const std::vector<Point> & obstacles, double last,
    double earliest_rejoin, double horizon, const builtin_interfaces::msg::Time & stamp,
    double clearance, double wall_clearance)
  {
    // Only replace the blocked portion and rejoin downstream on the same route.
    // The approach must turn toward the bypass before the first footprint hit.
    for (double lateral = std::min(0.45, max_lateral_); ;
      lateral = std::min(lateral + 0.05, max_lateral_))
    {
      for (double fall : {1.25, 1.45, 1.05, 0.85, 0.65}) {
        const double rejoin = std::max(last + fall, earliest_rejoin);
        if (rejoin > horizon) {
          continue;
        }
        for (double side : {1.0, -1.0}) {
          for (double rise : {1.05, 0.90, 1.25, 0.80}) {
            for (double entry_slope : {1.2, 1.5, 0.0}) {
              auto patch = makePatch(route_, start, progress_, rejoin, rise, fall,
                  side * lateral, entry_slope);
              if (patch.empty()) {
                continue;
              }
              const double yaw = patch.yaw(0.0);
              if (clearEntryTurn(*map_, start, yaw, obstacles, tracking_clearance_,
                wall_clearance) &&
                clearRoute(*map_, patch, 0.0, patch.distance.back(), obstacles,
                clearance, wall_clearance))
              {
                if (camera_stop_requested_) {
                  return hold("CAMERA_ESTOP");
                }
                detour_ = std::move(patch);
                detour_progress_ = 0.0;
                rejoin_ = rejoin;
                waiting_ = false;
                entry_pose_ = poseAt(detour_.path.header, xy(start), yaw);
                entry_pose_.header.stamp = stamp;
                return activeDetour(stamp);
              }
            }
          }
        }
      }
      if (lateral == max_lateral_) {
        break;
      }
    }
    return Path();
  }

  Path activeDetour(const builtin_interfaces::msg::Time & stamp)
  {
    if (camera_stop_requested_) {
      return hold("CAMERA_ESTOP");
    }
    entry_->publish(entry_pose_);
    publish("DETOURING");
    return withSuffix(detour_, detour_progress_, rejoin_, stamp);
  }

  Path withSuffix(
    const Route & patch, double from, double rejoin, const builtin_interfaces::msg::Time & stamp)
  {
    auto path = remaining(patch, from, stamp);
    auto suffix = remaining(route_, rejoin, stamp);
    path.poses.insert(path.poses.end(), suffix.poses.begin() + 1, suffix.poses.end());
    // At a rejoin coinciding with the goal, retain the goal's requested yaw.
    if (suffix.poses.size() == 1) {
      path.poses.back() = suffix.poses.back();
    }
    return path;
  }

  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader_{"nav2_core", "nav2_core::GlobalPlanner"};
  nav2_core::GlobalPlanner::Ptr baseline_;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr state_;
  rclcpp_lifecycle::LifecyclePublisher<PoseStamped>::SharedPtr entry_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacles_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr camera_stop_;
  std::atomic_bool camera_stop_requested_{false};
  std::shared_ptr<tf2_ros::Buffer> tf_;
  Costmap2D * map_{nullptr};
  std::mutex plan_mutex_, cloud_mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_;
  SteadyClock::time_point cloud_received_{}, blocked_since_{}, last_request_{};
  Route route_, detour_;
  PoseStamped entry_pose_;
  double progress_{}, detour_progress_{}, rejoin_{};
  double wait_{}, max_lateral_{}, max_length_{}, clearance_{}, tracking_clearance_{};
  double wall_clearance_{};
  std::vector<Point> remembered_obstacles_;
  bool waiting_{}, replacement_pending_{};
};
}  // namespace traymover_robot_nav

PLUGINLIB_EXPORT_CLASS(traymover_robot_nav::Option22LocalDetour, nav2_core::GlobalPlanner)
