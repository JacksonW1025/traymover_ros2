#ifndef LIDAR_LOCALIZATION__MAP_DOWNSAMPLING_HPP_
#define LIDAR_LOCALIZATION__MAP_DOWNSAMPLING_HPP_

#include <cmath>
#include <stdexcept>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lidar_localization
{

// Separate from scan voxel_leaf_size: filter the static target once, preserve
// its coordinates/intensity, and leave the source PCD and caller's cloud intact.
inline pcl::PointCloud<pcl::PointXYZI>::Ptr downsampleMapCloud(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & input, double leaf_size)
{
  if (!input || !std::isfinite(leaf_size) || leaf_size < 0.0) {
    throw std::invalid_argument("map_voxel_leaf_size must be finite and >= 0");
  }
  if (leaf_size == 0.0) {
    return input;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> filter;
  filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  filter.setDownsampleAllData(true);
  filter.setInputCloud(input);
  filter.filter(*output);
  return output;
}

}  // namespace lidar_localization

#endif  // LIDAR_LOCALIZATION__MAP_DOWNSAMPLING_HPP_
