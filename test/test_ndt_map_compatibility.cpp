#include "lidar_localization/ndt_map_compatibility.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace
{

struct Point
{
  float x;
  float y;
  float z;
};

bool expect(bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAILED: " << message << std::endl;
  }
  return condition;
}

lidar_localization::NdtResolutionPolicy default_policy()
{
  lidar_localization::NdtResolutionPolicy policy;
  policy.configured_resolution = 0.5;
  policy.max_resolution = 3.0;
  policy.resolution_step = 0.1;
  policy.min_points_per_voxel = 6;
  policy.min_usable_voxel_ratio = 0.01;
  return policy;
}

std::vector<Point> make_dense_cloud()
{
  std::vector<Point> points;
  for (int voxel = 0; voxel < 120; ++voxel) {
    const float base_x = static_cast<float>(voxel % 20);
    const float base_y = static_cast<float>(voxel / 20);
    for (int sample = 0; sample < 8; ++sample) {
      points.push_back(Point{
        base_x + 0.01f * static_cast<float>(sample),
        base_y + 0.01f * static_cast<float>(sample % 4),
        0.01f * static_cast<float>(sample % 3)});
    }
  }
  return points;
}

std::vector<Point> make_sparse_grid(double spacing)
{
  std::vector<Point> points;
  for (int x = 0; x < 30; ++x) {
    for (int y = 0; y < 30; ++y) {
      for (int z = 0; z < 4; ++z) {
        points.push_back(Point{
          static_cast<float>(spacing * x + 0.01),
          static_cast<float>(spacing * y + 0.01),
          static_cast<float>(spacing * z + 0.01)});
      }
    }
  }
  return points;
}

std::vector<Point> make_small_dense_cloud()
{
  std::vector<Point> points;
  for (int voxel = 0; voxel < 5; ++voxel) {
    for (int sample = 0; sample < 8; ++sample) {
      points.push_back(Point{
        static_cast<float>(voxel) + 0.01f * static_cast<float>(sample),
        0.01f * static_cast<float>(sample % 4),
        0.01f * static_cast<float>(sample % 3)});
    }
  }
  return points;
}

}  // namespace

int main()
{
  bool ok = true;

  const auto dense = make_dense_cloud();
  const auto dense_stats = lidar_localization::analyzeNdtMapVoxels(dense, 0.5, 6);
  ok &= expect(dense_stats.usable_voxels >= 100, "dense map has usable 0.5 m voxels");

  lidar_localization::NdtResolutionSelection selection;
  ok &= expect(
    lidar_localization::selectNdtResolution(dense, default_policy(), selection),
    "dense map is accepted");
  ok &= expect(
    std::abs(selection.resolution - 0.5) < 1e-9,
    "dense map keeps configured resolution");

  const auto small_dense = make_small_dense_cloud();
  ok &= expect(
    lidar_localization::selectNdtResolution(small_dense, default_policy(), selection),
    "small dense map is not rejected by an absolute voxel-count threshold");
  ok &= expect(
    std::abs(selection.resolution - 0.5) < 1e-9,
    "small dense map keeps configured resolution");

  const auto sparse = make_sparse_grid(1.0);
  const auto sparse_stats = lidar_localization::analyzeNdtMapVoxels(sparse, 0.5, 6);
  ok &= expect(sparse_stats.usable_voxels == 0, "sparse map has no usable 0.5 m voxels");
  ok &= expect(
    lidar_localization::selectNdtResolution(sparse, default_policy(), selection),
    "sparse map finds a coarser usable resolution");
  ok &= expect(selection.resolution > 0.5, "sparse map increases NDT resolution");
  ok &= expect(selection.resolution <= 3.0, "adaptive resolution respects maximum");
  ok &= expect(selection.stats.usable_voxels > 0, "selected resolution has usable voxels");
  ok &= expect(
    selection.stats.usableVoxelRatio() >= 0.01,
    "selected resolution has enough usable occupied voxels");

  const auto irrecoverably_sparse = make_sparse_grid(10.0);
  ok &= expect(
    !lidar_localization::selectNdtResolution(
      irrecoverably_sparse, default_policy(), selection),
    "map that stays sparse through the maximum resolution is rejected");

  return ok ? 0 : 1;
}
