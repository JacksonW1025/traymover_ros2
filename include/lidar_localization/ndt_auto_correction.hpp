#ifndef LIDAR_LOCALIZATION__NDT_AUTO_CORRECTION_HPP_
#define LIDAR_LOCALIZATION__NDT_AUTO_CORRECTION_HPP_

#include <algorithm>
#include <cmath>
#include <Eigen/Geometry>

namespace lidar_localization
{

struct AutoCorrectionPolicy
{
  int confirmations{3};
  double max_translation{2.5};
  double max_rotation{0.60};
  double consistency_translation{0.20};
  double consistency_rotation{0.10};
  double min_improvement{0.01};
  double max_step_translation{0.15};
  double max_step_rotation{0.08};
  double max_gap{6.0};
  double max_odom_speed{2.0};
  double max_odom_vertical_speed{0.5};
  double max_odom_angular_speed{1.5};

  bool valid() const
  {
    const double positive[] = {max_translation, max_rotation, consistency_translation,
      consistency_rotation, max_step_translation, max_step_rotation, max_gap,
      max_odom_speed, max_odom_vertical_speed, max_odom_angular_speed};
    for (double value : positive) {
      if (!std::isfinite(value) || value <= 0.0) {return false;}
    }
    return confirmations >= 2 && std::isfinite(min_improvement) &&
           min_improvement > 0.0 && min_improvement < 1.0 &&
           max_rotation <= 3.141592653589793 &&
           consistency_translation <= max_translation &&
           consistency_rotation <= max_rotation &&
           max_step_translation <= max_translation && max_step_rotation <= max_rotation;
  }
};

inline double rotationDistance(const Eigen::Affine3d & a, const Eigen::Affine3d & b)
{
  return std::abs(Eigen::AngleAxisd(a.rotation().transpose() * b.rotation()).angle());
}

inline bool withinCorrection(
  const Eigen::Affine3d & a, const Eigen::Affine3d & b, double translation, double rotation)
{
  return a.matrix().allFinite() && b.matrix().allFinite() &&
         (a.translation() - b.translation()).norm() <= translation &&
         rotationDistance(a, b) <= rotation;
}

// Odom discontinuities cannot be repaired by accepting a larger map match.
// Small fixed allowances cover measurement jitter, not sustained motion.
inline bool plausibleOdom(
  const Eigen::Affine3d & previous, const Eigen::Affine3d & current,
  double dt, const AutoCorrectionPolicy & policy)
{
  if (!previous.matrix().allFinite() || !current.matrix().allFinite() ||
    !std::isfinite(dt) || dt < 0.0 || dt > policy.max_gap)
  {
    return false;
  }
  return withinCorrection(previous, current,
    policy.max_odom_speed * dt + 0.15, policy.max_odom_angular_speed * dt + 0.15) &&
         std::abs(current.translation().z() - previous.translation().z()) <=
         policy.max_odom_vertical_speed * dt + 0.10;
}

struct CorrectionEvidence
{
  Eigen::Affine3d anchor{Eigen::Affine3d::Identity()};
  int count{0};
  double stamp{-1.0};

  void reset() {count = 0; stamp = -1.0;}

  bool confirm(
    const Eigen::Affine3d & candidate_map_odom, const Eigen::Affine3d & odom_at_scan,
    double scan_stamp, const AutoCorrectionPolicy & policy)
  {
    if (!candidate_map_odom.matrix().allFinite() || !odom_at_scan.matrix().allFinite() ||
      !std::isfinite(scan_stamp) || scan_stamp <= stamp)
    {
      reset();
      return false;
    }
    if (count == 0 || scan_stamp - stamp > policy.max_gap ||
      !withinCorrection(anchor * odom_at_scan, candidate_map_odom * odom_at_scan,
      policy.consistency_translation, policy.consistency_rotation))
    {
      anchor = candidate_map_odom;
      count = 1;
    } else {
      ++count;
    }
    stamp = scan_stamp;
    return count >= policy.confirmations;
  }
};

// Interpolate the robot pose at the application timestamp, not the TF origin.
inline Eigen::Affine3d boundedCorrection(
  const Eigen::Affine3d & old_map_odom, const Eigen::Affine3d & candidate_map_odom,
  const Eigen::Affine3d & odom_at_application, double smoothing,
  const AutoCorrectionPolicy & policy)
{
  const Eigen::Affine3d old_pose = old_map_odom * odom_at_application;
  const Eigen::Affine3d candidate_pose = candidate_map_odom * odom_at_application;
  const double distance = (candidate_pose.translation() - old_pose.translation()).norm();
  const double rotation = rotationDistance(old_pose, candidate_pose);
  double fraction = std::min(1.0, smoothing);
  if (distance > 0.0) {fraction = std::min(fraction, policy.max_step_translation / distance);}
  if (rotation > 0.0) {fraction = std::min(fraction, policy.max_step_rotation / rotation);}
  Eigen::Affine3d corrected = Eigen::Affine3d::Identity();
  corrected.translation() = old_pose.translation() +
    fraction * (candidate_pose.translation() - old_pose.translation());
  corrected.linear() = Eigen::Quaterniond(old_pose.rotation()).slerp(
    fraction, Eigen::Quaterniond(candidate_pose.rotation())).normalized().toRotationMatrix();
  return corrected * odom_at_application.inverse();
}

}  // namespace lidar_localization
#endif  // LIDAR_LOCALIZATION__NDT_AUTO_CORRECTION_HPP_
