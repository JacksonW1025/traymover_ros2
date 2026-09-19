#ifndef LIDAR_LOCALIZATION__WEIGHTED_FITNESS_HPP_
#define LIDAR_LOCALIZATION__WEIGHTED_FITNESS_HPP_

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_localization
{

struct WeightedFitness
{
  double baseline{std::numeric_limits<double>::max()};
  double weighted{std::numeric_limits<double>::max()};
  std::size_t searches{0};
};

// The weights represent copies in the registration source. Query each original
// point once and reuse its squared distance for both duplicate-cloud averages.
// Keep PCL's float transform and distance arithmetic; never change its source.
template<typename SearchTree>
WeightedFitness evaluateWeightedFitness(
  const pcl::PointCloud<pcl::PointXYZI> & unique,
  const std::vector<int> & baseline_weights,
  const std::vector<int> & weighted_weights,
  const Eigen::Matrix4f & transform, SearchTree & tree)
{
  WeightedFitness result;
  if (unique.empty() || baseline_weights.size() != unique.size() ||
    weighted_weights.size() != unique.size() || !transform.allFinite())
  {
    return result;
  }
  for (std::size_t i = 0; i < unique.size(); ++i) {
    if (baseline_weights[i] <= 0 || weighted_weights[i] < baseline_weights[i] ||
      !std::isfinite(unique[i].x) || !std::isfinite(unique[i].y) ||
      !std::isfinite(unique[i].z))
    {
      return result;
    }
  }

  pcl::PointCloud<pcl::PointXYZI> transformed;
  pcl::transformPointCloud(unique, transformed, transform);
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  double baseline_sum = 0.0;
  double weighted_sum = 0.0;
  double baseline_count = 0.0;
  double weighted_count = 0.0;
  for (std::size_t i = 0; i < transformed.size(); ++i) {
    const auto & point = transformed[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      return result;
    }
    ++result.searches;
    if (tree.nearestKSearch(point, 1, indices, distances) != 1 ||
      distances.empty() || !std::isfinite(distances[0]) || distances[0] < 0.0f)
    {
      return result;
    }
    const double distance = distances[0];
    baseline_sum += distance * baseline_weights[i];
    weighted_sum += distance * weighted_weights[i];
    baseline_count += baseline_weights[i];
    weighted_count += weighted_weights[i];
  }
  result.baseline = baseline_sum / baseline_count;
  result.weighted = weighted_sum / weighted_count;
  return result;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION__WEIGHTED_FITNESS_HPP_
