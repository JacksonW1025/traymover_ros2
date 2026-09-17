#ifndef LIDAR_LOCALIZATION__NDT_MAP_COMPATIBILITY_HPP_
#define LIDAR_LOCALIZATION__NDT_MAP_COMPATIBILITY_HPP_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>

namespace lidar_localization
{

struct NdtVoxelIndex
{
  std::int64_t x;
  std::int64_t y;
  std::int64_t z;

  bool operator==(const NdtVoxelIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct NdtVoxelIndexHash
{
  std::size_t operator()(const NdtVoxelIndex & index) const
  {
    std::size_t seed = std::hash<std::int64_t>{}(index.x);
    seed ^= std::hash<std::int64_t>{}(index.y) + 0x9e3779b9U + (seed << 6) + (seed >> 2);
    seed ^= std::hash<std::int64_t>{}(index.z) + 0x9e3779b9U + (seed << 6) + (seed >> 2);
    return seed;
  }
};

struct NdtVoxelStats
{
  std::size_t total_points{0};
  std::size_t total_voxels{0};
  std::size_t usable_points{0};
  std::size_t usable_voxels{0};
  std::size_t max_points_per_voxel{0};

  double usablePointRatio() const
  {
    if (total_points == 0) {
      return 0.0;
    }
    return static_cast<double>(usable_points) / static_cast<double>(total_points);
  }

  double usableVoxelRatio() const
  {
    if (total_voxels == 0) {
      return 0.0;
    }
    return static_cast<double>(usable_voxels) / static_cast<double>(total_voxels);
  }
};

struct NdtResolutionPolicy
{
  double configured_resolution{0.5};
  double max_resolution{3.0};
  double resolution_step{0.1};
  std::size_t min_points_per_voxel{6};
  double min_usable_voxel_ratio{0.01};
};

struct NdtResolutionSelection
{
  double resolution{0.0};
  NdtVoxelStats stats;
};

template<typename PointRange>
NdtVoxelStats analyzeNdtMapVoxels(
  const PointRange & points,
  double resolution,
  std::size_t min_points_per_voxel)
{
  NdtVoxelStats stats;
  if (!(resolution > 0.0) || min_points_per_voxel == 0) {
    return stats;
  }

  std::unordered_map<NdtVoxelIndex, std::size_t, NdtVoxelIndexHash> counts;
  counts.reserve(points.size());
  const double inverse_resolution = 1.0 / resolution;

  for (const auto & point : points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    const NdtVoxelIndex index{
      static_cast<std::int64_t>(std::floor(point.x * inverse_resolution)),
      static_cast<std::int64_t>(std::floor(point.y * inverse_resolution)),
      static_cast<std::int64_t>(std::floor(point.z * inverse_resolution))};
    ++counts[index];
    ++stats.total_points;
  }

  stats.total_voxels = counts.size();
  for (const auto & entry : counts) {
    const std::size_t count = entry.second;
    if (count > stats.max_points_per_voxel) {
      stats.max_points_per_voxel = count;
    }
    if (count >= min_points_per_voxel) {
      ++stats.usable_voxels;
      stats.usable_points += count;
    }
  }
  return stats;
}

inline bool ndtVoxelStatsMeetPolicy(
  const NdtVoxelStats & stats,
  const NdtResolutionPolicy & policy)
{
  return stats.usable_voxels > 0 &&
         stats.usableVoxelRatio() >= policy.min_usable_voxel_ratio;
}

template<typename PointRange>
bool selectNdtResolution(
  const PointRange & points,
  const NdtResolutionPolicy & policy,
  NdtResolutionSelection & selection)
{
  if (!(policy.configured_resolution > 0.0) ||
    policy.max_resolution < policy.configured_resolution ||
    !(policy.resolution_step > 0.0) ||
    policy.min_points_per_voxel == 0 ||
    policy.min_usable_voxel_ratio < 0.0 ||
    policy.min_usable_voxel_ratio > 1.0)
  {
    return false;
  }

  const double epsilon = policy.resolution_step * 1.0e-6;
  for (double resolution = policy.configured_resolution;
    resolution <= policy.max_resolution + epsilon;
    resolution += policy.resolution_step)
  {
    const NdtVoxelStats stats = analyzeNdtMapVoxels(
      points, resolution, policy.min_points_per_voxel);
    if (ndtVoxelStatsMeetPolicy(stats, policy)) {
      selection.resolution = resolution;
      selection.stats = stats;
      return true;
    }
  }
  return false;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION__NDT_MAP_COMPATIBILITY_HPP_
