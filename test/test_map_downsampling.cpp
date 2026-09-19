#include "lidar_localization/map_downsampling.hpp"
#include "lidar_localization/ndt_map_compatibility.hpp"

#include <cmath>
#include <iostream>
#include <limits>

namespace
{
bool expect(bool value, const char * message)
{
  if (!value) {
    std::cerr << "FAILED: " << message << std::endl;
  }
  return value;
}
}

int main()
{
  bool ok = true;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  // Two close returns in each fine voxel, with enough fine voxels per NDT cell
  // to retain the >=6 samples required by the NDT covariance estimator.
  for (int x = 0; x < 3; ++x) {
    for (int y = 0; y < 3; ++y) {
      for (int z = 0; z < 2; ++z) {
        for (int repeat = 0; repeat < 2; ++repeat) {
          pcl::PointXYZI p;
          p.x = 0.15f * x + 0.02f + repeat * 0.01f;
          p.y = 0.15f * y + 0.02f;
          p.z = 0.15f * z + 0.02f;
          p.intensity = 2.0f + 2.0f * repeat;
          cloud->push_back(p);
        }
      }
    }
  }
  const auto original = *cloud;
  const auto filtered = lidar_localization::downsampleMapCloud(cloud, 0.15);
  ok &= expect(filtered->size() == 18, "one centroid per map voxel");
  ok &= expect(cloud->size() == original.size(), "source cloud size unchanged");
  for (std::size_t i = 0; i < cloud->size(); ++i) {
    ok &= expect(cloud->points[i].x == original.points[i].x, "source coordinates unchanged");
  }
  for (const auto & p : *filtered) {
    const double x_cell = (p.x - 0.025) / 0.15;
    ok &= expect(std::abs(x_cell - std::round(x_cell)) < 1e-5, "centroid remains in map frame");
    ok &= expect(std::abs(p.intensity - 3.0f) < 1e-5, "intensity averaged");
  }
  const auto stats = lidar_localization::analyzeNdtMapVoxels(filtered->points, 0.5, 6);
  ok &= expect(stats.usable_voxels == 1 && stats.usable_points == 18,
    "downsampled cloud retains usable NDT covariance voxel");
  ok &= expect(lidar_localization::downsampleMapCloud(cloud, 0.0) == cloud,
    "zero leaf size disables filtering");
  for (double invalid : {-0.1, std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity()})
  {
    bool rejected = false;
    try {
      lidar_localization::downsampleMapCloud(cloud, invalid);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    ok &= expect(rejected, "invalid leaf size rejected");
  }
  return ok ? 0 : 1;
}
