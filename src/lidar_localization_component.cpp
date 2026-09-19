#include <lidar_localization/lidar_localization_component.hpp>
#include <lidar_localization/ndt_map_compatibility.hpp>
#include <lidar_localization/map_downsampling.hpp>
#include <lidar_localization/weighted_fitness.hpp>
#include <algorithm>
#include <boost/pointer_cast.hpp>
#include <chrono>
#include <limits>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>

namespace {

bool has_field(const pcl::PCLPointCloud2 & cloud, const std::string & name)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      return true;
    }
  }
  return false;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr to_xyzi_cloud(
  const pcl::PCLPointCloud2 & raw_cloud,
  rclcpp::Logger logger,
  const std::string & source_name)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  if (has_field(raw_cloud, "intensity")) {
    pcl::fromPCLPointCloud2(raw_cloud, *xyzi_cloud);
    return xyzi_cloud;
  }

  pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
  pcl::fromPCLPointCloud2(raw_cloud, xyz_cloud);
  xyzi_cloud->header = xyz_cloud.header;
  xyzi_cloud->width = xyz_cloud.width;
  xyzi_cloud->height = xyz_cloud.height;
  xyzi_cloud->is_dense = xyz_cloud.is_dense;
  xyzi_cloud->points.resize(xyz_cloud.points.size());
  for (size_t i = 0; i < xyz_cloud.points.size(); ++i) {
    xyzi_cloud->points[i].x = xyz_cloud.points[i].x;
    xyzi_cloud->points[i].y = xyz_cloud.points[i].y;
    xyzi_cloud->points[i].z = xyz_cloud.points[i].z;
    xyzi_cloud->points[i].intensity = 0.0f;
  }
  RCLCPP_WARN(
    logger,
    "%s has no intensity field; using synthetic intensity=0 for NDT compatibility",
    source_name.c_str());
  return xyzi_cloud;
}

double yaw_from_matrix(const Eigen::Matrix4f & transform)
{
  const Eigen::Matrix3f rotation = transform.block<3, 3>(0, 0);
  return std::atan2(rotation(1, 0), rotation(0, 0));
}

void log_cloud_bounds(
  rclcpp::Logger logger,
  const std::string & label,
  const pcl::PointCloud<pcl::PointXYZI> & cloud)
{
  if (cloud.empty()) {
    RCLCPP_WARN(logger, "%s cloud is empty", label.c_str());
    return;
  }

  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float min_z = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::lowest();
  float max_y = std::numeric_limits<float>::lowest();
  float max_z = std::numeric_limits<float>::lowest();

  for (const auto & point : cloud.points) {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    min_z = std::min(min_z, point.z);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
    max_z = std::max(max_z, point.z);
  }

  RCLCPP_INFO(
    logger,
    "%s cloud: points=%zu x=[%.2f, %.2f] y=[%.2f, %.2f] z=[%.2f, %.2f]",
    label.c_str(), cloud.size(), min_x, max_x, min_y, max_y, min_z, max_z);
}

}  // namespace

PCLLocalization::PCLLocalization(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("lidar_localization", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  tflistener_(tfbuffer_),
  broadcaster_(this)
{
  declare_parameter("global_frame_id", "map");
  tf_publish_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  declare_parameter("odom_frame_id", "odom");
  declare_parameter("base_frame_id", "base_link");
  declare_parameter("enable_map_odom_tf", false);
  declare_parameter("registration_method", "NDT");
  declare_parameter("score_threshold", 2.0);
  declare_parameter("ndt_resolution", 1.0);
  declare_parameter("ndt_step_size", 0.1);
  declare_parameter("ndt_max_iterations", 35);
  declare_parameter("ndt_num_threads", 4);
  declare_parameter("adaptive_ndt_resolution", true);
  declare_parameter("adaptive_ndt_max_resolution", 3.0);
  declare_parameter("adaptive_ndt_resolution_step", 0.1);
  declare_parameter("adaptive_ndt_min_usable_voxel_ratio", 0.01);
  declare_parameter("transform_epsilon", 0.01);
  declare_parameter("voxel_leaf_size", 0.2);
  declare_parameter("map_voxel_leaf_size", 0.0);
  declare_parameter("scan_max_range", 100.0);
  declare_parameter("scan_min_range", 1.0);
  declare_parameter("far_point_boost_min_range", 0.0);
  declare_parameter("far_point_boost_factor", 1);
  declare_parameter("height_boost_enabled", false);
  declare_parameter("height_boost_min_z", 2.0);
  declare_parameter("auto_correction_enabled", false);
  declare_parameter("auto_correction_confirmations", auto_correction_policy_.confirmations);
  declare_parameter("auto_correction_max_translation", auto_correction_policy_.max_translation);
  declare_parameter("auto_correction_max_rotation", auto_correction_policy_.max_rotation);
  declare_parameter("auto_correction_consistency_translation", auto_correction_policy_.consistency_translation);
  declare_parameter("auto_correction_consistency_rotation", auto_correction_policy_.consistency_rotation);
  declare_parameter("auto_correction_min_improvement", auto_correction_policy_.min_improvement);
  declare_parameter("auto_correction_max_step_translation", auto_correction_policy_.max_step_translation);
  declare_parameter("auto_correction_max_step_rotation", auto_correction_policy_.max_step_rotation);
  declare_parameter("auto_correction_max_gap", auto_correction_policy_.max_gap);
  declare_parameter("auto_correction_max_odom_speed", auto_correction_policy_.max_odom_speed);
  declare_parameter("auto_correction_max_odom_vertical_speed", auto_correction_policy_.max_odom_vertical_speed);
  declare_parameter("auto_correction_max_odom_angular_speed", auto_correction_policy_.max_odom_angular_speed);
  declare_parameter("scan_period", 0.1);
  declare_parameter("use_pcd_map", false);
  declare_parameter("map_path", "/map/map.pcd");
  declare_parameter("set_initial_pose", false);
  declare_parameter("initial_pose_x", 0.0);
  declare_parameter("initial_pose_y", 0.0);
  declare_parameter("initial_pose_z", 0.0);
  declare_parameter("initial_pose_qx", 0.0);
  declare_parameter("initial_pose_qy", 0.0);
  declare_parameter("initial_pose_qz", 0.0);
  declare_parameter("initial_pose_qw", 1.0);
  declare_parameter("use_odom", false);
  declare_parameter("use_imu", false);
  declare_parameter("enable_debug", false);
  declare_parameter("enable_timer_publishing", false);
  declare_parameter("pose_publish_frequency", 10.0);
  declare_parameter("lock_planar", false);
  declare_parameter("max_pose_jump_translation", 1.0);
  declare_parameter("max_pose_jump_rotation", 0.5);
  declare_parameter("max_bootstrap_pose_jump_translation", -1.0);
  declare_parameter("max_bootstrap_pose_jump_rotation", -1.0);
  declare_parameter("ndt_align_interval_s", 5.0);
  declare_parameter("scan_odom_tf_timeout", 0.5);
  declare_parameter("tf_rebroadcast_rate_hz", 50.0);
  declare_parameter("map_odom_smoothing", 0.3);
  declare_parameter("max_map_odom_update_translation", -1.0);
  declare_parameter("max_map_odom_update_rotation", -1.0);
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn PCLLocalization::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  localization_active_ = false;
  updateTfSnapshot(false);
  initializeParameters();
  if (!std::isfinite(scan_odom_tf_timeout_) || scan_odom_tf_timeout_ <= 0.0 ||
    scan_odom_tf_timeout_ > 2.0)
  {
    RCLCPP_ERROR(get_logger(), "scan_odom_tf_timeout must be finite and in (0, 2] seconds");
    return CallbackReturn::FAILURE;
  }
  if (auto_correction_enabled_ && (!auto_correction_policy_.valid() ||
    !std::isfinite(map_odom_smoothing_) || map_odom_smoothing_ <= 0.0 ||
    map_odom_smoothing_ > 1.0 || !std::isfinite(max_pose_jump_translation_) ||
    max_pose_jump_translation_ <= 0.0 || !std::isfinite(max_pose_jump_rotation_) ||
    max_pose_jump_rotation_ <= 0.0 || enable_timer_publishing_))
  {
    RCLCPP_ERROR(get_logger(), "Invalid auto-correction policy: positive finite bounds, "
      "0 < smoothing <= 1, and scan-driven pose publishing are required");
    return CallbackReturn::FAILURE;
  }
  if (!std::isfinite(height_boost_min_z_) || height_boost_min_z_ < 0.0) {
    RCLCPP_ERROR(get_logger(), "height_boost_min_z must be finite and >= 0 (base frame metres)");
    return CallbackReturn::FAILURE;
  }
  if (!std::isfinite(map_voxel_leaf_size_) || map_voxel_leaf_size_ < 0.0) {
    RCLCPP_ERROR(get_logger(), "map_voxel_leaf_size must be finite and >= 0");
    return CallbackReturn::FAILURE;
  }
  initializePubSub();
  initializeRegistration();

  path_ptr_ = std::make_shared<nav_msgs::msg::Path>();
  path_ptr_->header.frame_id = global_frame_id_;

  RCLCPP_INFO(get_logger(), "Configuring end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");
  localization_active_ = false;
  updateTfSnapshot(false);

  pose_pub_->on_activate();
  path_pub_->on_activate();
  initial_map_pub_->on_activate();
  ndt_aligned_scan_pub_->on_activate();

  if (set_initial_pose_) {
    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.pose.position.x = initial_pose_x_;
    msg->pose.pose.position.y = initial_pose_y_;
    msg->pose.pose.position.z = initial_pose_z_;
    msg->pose.pose.orientation.x = initial_pose_qx_;
    msg->pose.pose.orientation.y = initial_pose_qy_;
    msg->pose.pose.orientation.z = initial_pose_qz_;
    msg->pose.pose.orientation.w = initial_pose_qw_;

    geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped(new geometry_msgs::msg::PoseStamped);
    pose_stamped->header.stamp = msg->header.stamp;
    pose_stamped->header.frame_id = global_frame_id_;
    pose_stamped->pose = msg->pose.pose;
    path_ptr_->poses.push_back(*pose_stamped);

    initialPoseReceived(msg);
  }

  if (use_pcd_map_) {
    pcl::PCLPointCloud2 raw_map_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr;
    // Load the map into a generic PCL container first so legacy XYZ/normal
    // maps can be upgraded to XYZI for the registration backend.
    if (map_path_.rfind(".pcd") != std::string::npos) {
      RCLCPP_INFO(get_logger(), "Loading pcd map from: %s", map_path_.c_str());
      if (pcl::io::loadPCDFile(map_path_, raw_map_cloud) == -1) {
        RCLCPP_ERROR(get_logger(), "Failed to load pcd file: %s", map_path_.c_str());
        return CallbackReturn::FAILURE;
      }
    } else if (map_path_.rfind(".ply") != std::string::npos) {
      RCLCPP_INFO(get_logger(), "Loading ply map from: %s", map_path_.c_str());
      if (pcl::io::loadPLYFile(map_path_, raw_map_cloud) == -1) {
        RCLCPP_ERROR(get_logger(), "Failed to load ply file: %s", map_path_.c_str());
        return CallbackReturn::FAILURE;
      }
    } else {
      RCLCPP_ERROR(
          get_logger(), "Unsupported map file format. Please use .pcd or .ply: %s",
          map_path_.c_str());
      return CallbackReturn::FAILURE;
    }
    map_cloud_ptr = to_xyzi_cloud(raw_map_cloud, get_logger(), map_path_);
    raw_map_cloud.data.clear();
    raw_map_cloud.data.shrink_to_fit();

    RCLCPP_INFO(get_logger(), "Map Size %ld", map_cloud_ptr->size());
    if (!configureRegistrationTarget(map_cloud_ptr)) {
      return CallbackReturn::FAILURE;
    }
    log_cloud_bounds(get_logger(), "NDT target map", *map_cloud_ptr);
    sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*map_cloud_ptr, *map_msg_ptr);
    map_msg_ptr->header.frame_id = global_frame_id_;
    initial_map_pub_->publish(*map_msg_ptr);
    RCLCPP_INFO(get_logger(), "Initial Map Published");

    map_recieved_ = true;
  }

  // High-frequency TF rebroadcaster. Keeps map→odom fresh independent of
  // LiDAR scan rate, so downstream consumers (RViz RobotModel, Nav2
  // costmaps) see the latest FAST_LIO pose through a fresh TF chain.
  if (tf_rebroadcast_rate_hz_ > 0.0) {
    const auto period = std::chrono::duration<double>(1.0 / tf_rebroadcast_rate_hz_);
    tf_rebroadcast_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PCLLocalization::rebroadcastMapOdomTF, this), tf_publish_group_);
  }
  localization_active_ = true;
  updateTfSnapshot(have_good_pose_);

  RCLCPP_INFO(get_logger(), "Activating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  localization_active_ = false;
  updateTfSnapshot(false);

  pose_pub_->on_deactivate();
  path_pub_->on_deactivate();
  initial_map_pub_->on_deactivate();
  ndt_aligned_scan_pub_->on_deactivate();

  if (tf_rebroadcast_timer_) {
    tf_rebroadcast_timer_->cancel();
    tf_rebroadcast_timer_.reset();
  }

  RCLCPP_INFO(get_logger(), "Deactivating end");
  return CallbackReturn::SUCCESS;
}

void PCLLocalization::updateTfSnapshot(bool valid)
{
  std::lock_guard<std::mutex> lock(tf_snapshot_mutex_);
  tf_snapshot_valid_ = valid && localization_active_;
  if (tf_snapshot_valid_) {
    tf_snapshot_ = tf2::eigenToTransform(Eigen::Affine3d(last_good_map_to_odom_.cast<double>()));
    tf_snapshot_.header.frame_id = global_frame_id_;
    tf_snapshot_.child_frame_id = odom_frame_id_;
  }
}

void PCLLocalization::rebroadcastMapOdomTF()
{
  // Keep the lock through publish so invalidation cannot race a stale send.
  // No optimizer, pose state, or map cloud is accessed by this callback.
  std::lock_guard<std::mutex> lock(tf_snapshot_mutex_);
  if (!tf_snapshot_valid_) {return;}
  tf_snapshot_.header.stamp = now();
  broadcaster_.sendTransform(tf_snapshot_);
}

CallbackReturn PCLLocalization::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning Up");
  localization_active_ = false;
  updateTfSnapshot(false);
  if (tf_rebroadcast_timer_) {
    tf_rebroadcast_timer_->cancel();
    tf_rebroadcast_timer_.reset();
  }
  initial_pose_sub_.reset();
  initial_map_pub_.reset();
  ndt_aligned_scan_pub_.reset();
  path_pub_.reset();
  pose_pub_.reset();
  odom_sub_.reset();
  cloud_sub_.reset();
  imu_sub_.reset();

  if (enable_timer_publishing_){
    pose_publish_timer_.reset();
  }

  RCLCPP_INFO(get_logger(), "Cleaning Up end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());
  localization_active_ = false;
  updateTfSnapshot(false);

  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_FATAL(get_logger(), "Error Processing from %s", state.label().c_str());
  localization_active_ = false;
  updateTfSnapshot(false);

  return CallbackReturn::SUCCESS;
}

void PCLLocalization::initializeParameters()
{
  RCLCPP_INFO(get_logger(), "initializeParameters");
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("enable_map_odom_tf", enable_map_odom_tf_);
  get_parameter("registration_method", registration_method_);
  get_parameter("score_threshold", score_threshold_);
  get_parameter("ndt_resolution", ndt_resolution_);
  get_parameter("ndt_step_size", ndt_step_size_);
  get_parameter("ndt_num_threads", ndt_num_threads_);
  get_parameter("ndt_max_iterations", ndt_max_iterations_);
  get_parameter("adaptive_ndt_resolution", adaptive_ndt_resolution_);
  get_parameter("adaptive_ndt_max_resolution", adaptive_ndt_max_resolution_);
  get_parameter("adaptive_ndt_resolution_step", adaptive_ndt_resolution_step_);
  get_parameter(
    "adaptive_ndt_min_usable_voxel_ratio", adaptive_ndt_min_usable_voxel_ratio_);
  get_parameter("transform_epsilon", transform_epsilon_);
  get_parameter("voxel_leaf_size", voxel_leaf_size_);
  get_parameter("map_voxel_leaf_size", map_voxel_leaf_size_);
  RCLCPP_INFO(get_logger(), "map_voxel_leaf_size: %.3f m", map_voxel_leaf_size_);
  get_parameter("scan_max_range", scan_max_range_);
  get_parameter("scan_min_range", scan_min_range_);
  get_parameter("far_point_boost_min_range", far_point_boost_min_range_);
  get_parameter("far_point_boost_factor", far_point_boost_factor_);
  get_parameter("height_boost_enabled", height_boost_enabled_);
  get_parameter("height_boost_min_z", height_boost_min_z_);
  get_parameter("auto_correction_enabled", auto_correction_enabled_);
  get_parameter("auto_correction_confirmations", auto_correction_policy_.confirmations);
  get_parameter("auto_correction_max_translation", auto_correction_policy_.max_translation);
  get_parameter("auto_correction_max_rotation", auto_correction_policy_.max_rotation);
  get_parameter("auto_correction_consistency_translation", auto_correction_policy_.consistency_translation);
  get_parameter("auto_correction_consistency_rotation", auto_correction_policy_.consistency_rotation);
  get_parameter("auto_correction_min_improvement", auto_correction_policy_.min_improvement);
  get_parameter("auto_correction_max_step_translation", auto_correction_policy_.max_step_translation);
  get_parameter("auto_correction_max_step_rotation", auto_correction_policy_.max_step_rotation);
  get_parameter("auto_correction_max_gap", auto_correction_policy_.max_gap);
  get_parameter("auto_correction_max_odom_speed", auto_correction_policy_.max_odom_speed);
  get_parameter("auto_correction_max_odom_vertical_speed", auto_correction_policy_.max_odom_vertical_speed);
  get_parameter("auto_correction_max_odom_angular_speed", auto_correction_policy_.max_odom_angular_speed);
  RCLCPP_INFO(get_logger(), "auto_correction_enabled: %d; confirmations=%d "
    "absolute=%.2fm/%.2frad step=%.2fm/%.2frad improvement=%.3f",
    auto_correction_enabled_, auto_correction_policy_.confirmations,
    auto_correction_policy_.max_translation, auto_correction_policy_.max_rotation,
    auto_correction_policy_.max_step_translation, auto_correction_policy_.max_step_rotation,
    auto_correction_policy_.min_improvement);
  get_parameter("scan_period", scan_period_);
  get_parameter("use_pcd_map", use_pcd_map_);
  get_parameter("map_path", map_path_);
  get_parameter("set_initial_pose", set_initial_pose_);
  get_parameter("initial_pose_x", initial_pose_x_);
  get_parameter("initial_pose_y", initial_pose_y_);
  get_parameter("initial_pose_z", initial_pose_z_);
  get_parameter("initial_pose_qx", initial_pose_qx_);
  get_parameter("initial_pose_qy", initial_pose_qy_);
  get_parameter("initial_pose_qz", initial_pose_qz_);
  get_parameter("initial_pose_qw", initial_pose_qw_);
  get_parameter("use_odom", use_odom_);
  get_parameter("use_imu", use_imu_);
  get_parameter("enable_debug", enable_debug_);
  get_parameter("enable_timer_publishing", enable_timer_publishing_);
  get_parameter("pose_publish_frequency", pose_publish_frequency_);
  get_parameter("lock_planar", lock_planar_);
  get_parameter("max_pose_jump_translation", max_pose_jump_translation_);
  get_parameter("max_pose_jump_rotation", max_pose_jump_rotation_);
  get_parameter("max_bootstrap_pose_jump_translation", max_bootstrap_pose_jump_translation_);
  get_parameter("max_bootstrap_pose_jump_rotation", max_bootstrap_pose_jump_rotation_);
  get_parameter("ndt_align_interval_s", ndt_align_interval_s_);
  get_parameter("scan_odom_tf_timeout", scan_odom_tf_timeout_);
  get_parameter("tf_rebroadcast_rate_hz", tf_rebroadcast_rate_hz_);
  get_parameter("map_odom_smoothing", map_odom_smoothing_);
  get_parameter("max_map_odom_update_translation", max_map_odom_update_translation_);
  get_parameter("max_map_odom_update_rotation", max_map_odom_update_rotation_);
  RCLCPP_INFO(get_logger(), "lock_planar: %d", lock_planar_);
  RCLCPP_INFO(get_logger(), "max_pose_jump_translation: %.2f m", max_pose_jump_translation_);
  RCLCPP_INFO(get_logger(), "max_pose_jump_rotation: %.2f rad", max_pose_jump_rotation_);
  RCLCPP_INFO(get_logger(), "bootstrap jump limits: translation=%.2f m rotation=%.2f rad",
    max_bootstrap_pose_jump_translation_, max_bootstrap_pose_jump_rotation_);
  RCLCPP_INFO(get_logger(), "ndt_align_interval_s: %.2f s", ndt_align_interval_s_);
  RCLCPP_INFO(get_logger(), "tf_rebroadcast_rate_hz: %.2f Hz", tf_rebroadcast_rate_hz_);
  RCLCPP_INFO(get_logger(), "map_odom_smoothing: %.2f", map_odom_smoothing_);
  RCLCPP_INFO(
    get_logger(), "max_map_odom_update_translation: %.2f m",
    max_map_odom_update_translation_);
  RCLCPP_INFO(
    get_logger(), "max_map_odom_update_rotation: %.2f rad",
    max_map_odom_update_rotation_);

  RCLCPP_INFO(get_logger(),"global_frame_id: %s", global_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"odom_frame_id: %s", odom_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"base_frame_id: %s", base_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"enable_map_odom_tf: %d", enable_map_odom_tf_);
  RCLCPP_INFO(get_logger(),"registration_method: %s", registration_method_.c_str());
  RCLCPP_INFO(get_logger(),"ndt_resolution: %lf", ndt_resolution_);
  RCLCPP_INFO(get_logger(),"ndt_step_size: %lf", ndt_step_size_);
  RCLCPP_INFO(get_logger(),"ndt_num_threads: %d", ndt_num_threads_);
  RCLCPP_INFO(get_logger(), "adaptive_ndt_resolution: %d", adaptive_ndt_resolution_);
  RCLCPP_INFO(
    get_logger(), "adaptive_ndt_max_resolution: %.2f m", adaptive_ndt_max_resolution_);
  RCLCPP_INFO(
    get_logger(), "adaptive_ndt_resolution_step: %.2f m", adaptive_ndt_resolution_step_);
  RCLCPP_INFO(
    get_logger(), "adaptive_ndt_min_usable_voxel_ratio: %.3f",
    adaptive_ndt_min_usable_voxel_ratio_);
  RCLCPP_INFO(get_logger(),"transform_epsilon: %lf", transform_epsilon_);
  RCLCPP_INFO(get_logger(),"voxel_leaf_size: %lf", voxel_leaf_size_);
  RCLCPP_INFO(get_logger(),"scan_max_range: %lf", scan_max_range_);
  RCLCPP_INFO(get_logger(),"scan_min_range: %lf", scan_min_range_);
  RCLCPP_INFO(get_logger(),"far_point_boost_min_range: %lf", far_point_boost_min_range_);
  RCLCPP_INFO(get_logger(),"far_point_boost_factor: %d", far_point_boost_factor_);
  RCLCPP_INFO(
    get_logger(), "height_boost: enabled=%d min_z=%.3f m (base frame) extra_copies=1 "
    "max_copies=3 (existing far weighting preserved); fitness gate uses baseline cloud",
    height_boost_enabled_, height_boost_min_z_);
  RCLCPP_INFO(get_logger(),"scan_period: %lf", scan_period_);
  RCLCPP_INFO(get_logger(),"use_pcd_map: %d", use_pcd_map_);
  RCLCPP_INFO(get_logger(),"map_path: %s", map_path_.c_str());
  RCLCPP_INFO(get_logger(),"set_initial_pose: %d", set_initial_pose_);
  RCLCPP_INFO(get_logger(),"use_odom: %d", use_odom_);
  RCLCPP_INFO(get_logger(),"use_imu: %d", use_imu_);
  RCLCPP_INFO(get_logger(),"enable_debug: %d", enable_debug_);
  RCLCPP_INFO(get_logger(),"enable_timer_publishing: %d", enable_timer_publishing_);
  RCLCPP_INFO(get_logger(),"pose_publish_frequency: %lf", pose_publish_frequency_);
}

void PCLLocalization::initializePubSub()
{
  RCLCPP_INFO(get_logger(), "initializePubSub");

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pcl_pose",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "path",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "initial_map",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  ndt_aligned_scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "ndt_aligned_scan",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::SystemDefaultsQoS(),
    std::bind(&PCLLocalization::initialPoseReceived, this, std::placeholders::_1));

  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "ndt_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&PCLLocalization::mapReceived, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::odomReceived, this, std::placeholders::_1));

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "cloud", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::cloudReceived, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::imuReceived, this, std::placeholders::_1));

  if (enable_timer_publishing_) {
    auto period = std::chrono::duration<double>(1.0 / pose_publish_frequency_);
    pose_publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PCLLocalization::timerPublishPose, this));
  }

  RCLCPP_INFO(get_logger(), "initializePubSub end");
}

void PCLLocalization::initializeRegistration()
{
  RCLCPP_INFO(get_logger(), "initializeRegistration");

  if (registration_method_ == "GICP") {
    boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>> gicp(
      new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp;
  }
  else if (registration_method_ == "NDT") {
    boost::shared_ptr<pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ndt(
      new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setStepSize(ndt_step_size_);
    ndt->setResolution(ndt_resolution_);
    ndt->setTransformationEpsilon(transform_epsilon_);
    registration_ = ndt;
  }
  else if (registration_method_ == "NDT_OMP") {
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(
      new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt_omp->setStepSize(ndt_step_size_);
    ndt_omp->setResolution(ndt_resolution_);
    ndt_omp->setTransformationEpsilon(transform_epsilon_);
    if (ndt_num_threads_ > 0) {
      ndt_omp->setNumThreads(ndt_num_threads_);
    } else {
      ndt_omp->setNumThreads(omp_get_max_threads());
    }
    registration_ = ndt_omp;
  }
  else if (registration_method_ == "GICP_OMP") {
    pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp_omp(
      new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp_omp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp_omp;
  }
  else {
    RCLCPP_ERROR(get_logger(), "Invalid registration method.");
    exit(EXIT_FAILURE);
  }
  registration_->setMaximumIterations(ndt_max_iterations_);


  voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  RCLCPP_INFO(get_logger(), "initializeRegistration end");
}

bool PCLLocalization::setEffectiveNdtResolution(double resolution)
{
  if (registration_method_ == "NDT") {
    auto ndt = boost::dynamic_pointer_cast<
      pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>(registration_);
    if (!ndt) {
      RCLCPP_ERROR(get_logger(), "Failed to access NDT registration backend");
      return false;
    }
    ndt->setResolution(resolution);
  } else if (registration_method_ == "NDT_OMP") {
    auto ndt = boost::dynamic_pointer_cast<
      pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>(registration_);
    if (!ndt) {
      RCLCPP_ERROR(get_logger(), "Failed to access NDT_OMP registration backend");
      return false;
    }
    ndt->setResolution(static_cast<float>(resolution));
  } else {
    return true;
  }

  return true;
}

bool PCLLocalization::configureNdtResolutionForMap(
  const pcl::PointCloud<pcl::PointXYZI> & map_cloud)
{
  if (registration_method_ != "NDT" && registration_method_ != "NDT_OMP") {
    return true;
  }

  constexpr std::size_t kNdtMinPointsPerVoxel = 6;
  lidar_localization::NdtResolutionPolicy policy;
  policy.configured_resolution = ndt_resolution_;
  policy.max_resolution = adaptive_ndt_max_resolution_;
  policy.resolution_step = adaptive_ndt_resolution_step_;
  policy.min_points_per_voxel = kNdtMinPointsPerVoxel;
  policy.min_usable_voxel_ratio = adaptive_ndt_min_usable_voxel_ratio_;

  const auto configured_stats = lidar_localization::analyzeNdtMapVoxels(
    map_cloud.points, ndt_resolution_, kNdtMinPointsPerVoxel);
  RCLCPP_INFO(
    get_logger(),
    "NDT map density at %.2f m: points=%zu voxels=%zu usable_voxels=%zu "
    "usable_points=%zu (%.1f%%), usable_voxel_ratio=%.2f%%, "
    "max_points_per_voxel=%zu",
    ndt_resolution_, configured_stats.total_points, configured_stats.total_voxels,
    configured_stats.usable_voxels, configured_stats.usable_points,
    100.0 * configured_stats.usablePointRatio(),
    100.0 * configured_stats.usableVoxelRatio(), configured_stats.max_points_per_voxel);

  if (lidar_localization::ndtVoxelStatsMeetPolicy(configured_stats, policy)) {
    return setEffectiveNdtResolution(ndt_resolution_);
  }

  if (!adaptive_ndt_resolution_) {
    RCLCPP_ERROR(
      get_logger(),
      "NDT target is too sparse for configured resolution %.2f m and adaptive "
      "resolution is disabled",
      ndt_resolution_);
    return false;
  }

  lidar_localization::NdtResolutionSelection selection;
  if (!lidar_localization::selectNdtResolution(map_cloud.points, policy, selection)) {
    const auto max_stats = lidar_localization::analyzeNdtMapVoxels(
      map_cloud.points, adaptive_ndt_max_resolution_, kNdtMinPointsPerVoxel);
    RCLCPP_ERROR(
      get_logger(),
      "NDT target remains too sparse through %.2f m: usable_voxels=%zu, "
      "usable_voxel_ratio=%.2f%%, usable_points=%.1f%%; need at least one usable "
      "voxel and %.2f%% of occupied voxels. "
      "Use a denser PCD or raise adaptive_ndt_max_resolution.",
      adaptive_ndt_max_resolution_, max_stats.usable_voxels,
      100.0 * max_stats.usableVoxelRatio(), 100.0 * max_stats.usablePointRatio(),
      100.0 * adaptive_ndt_min_usable_voxel_ratio_);
    return false;
  }

  RCLCPP_WARN(
    get_logger(),
    "Sparse NDT target: increasing resolution from %.2f m to %.2f m "
    "(usable_voxels=%zu, usable_voxel_ratio=%.2f%%, usable_points=%.1f%%, "
    "max_points_per_voxel=%zu)",
    ndt_resolution_, selection.resolution, selection.stats.usable_voxels,
    100.0 * selection.stats.usableVoxelRatio(),
    100.0 * selection.stats.usablePointRatio(), selection.stats.max_points_per_voxel);
  return setEffectiveNdtResolution(selection.resolution);
}

bool PCLLocalization::configureRegistrationTarget(
  pcl::PointCloud<pcl::PointXYZI>::Ptr & map_cloud_ptr)
{
  const auto started = std::chrono::steady_clock::now();
  const auto original_points = map_cloud_ptr->size();
  auto target = lidar_localization::downsampleMapCloud(map_cloud_ptr, map_voxel_leaf_size_);
  if (target->empty()) {
    RCLCPP_ERROR(get_logger(), "Registration target is empty after map downsampling");
    return false;
  }
  const double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started).count();
  RCLCPP_INFO(get_logger(),
    "Map downsample: leaf=%.3f m input=%zu output=%zu retained=%.1f%% elapsed=%.3f s",
    map_voxel_leaf_size_, original_points, target->size(),
    100.0 * target->size() / original_points, elapsed);
  if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());
    voxel_grid_filter_.setInputCloud(target);
    voxel_grid_filter_.filter(*filtered_cloud_ptr);
    registration_->setInputTarget(filtered_cloud_ptr);
    map_cloud_ptr = filtered_cloud_ptr;
    return true;
  }

  if (!configureNdtResolutionForMap(*target)) {
    return false;
  }
  registration_->setInputTarget(target);
  map_cloud_ptr = target;
  return true;
}

void PCLLocalization::initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "initialPoseReceived");
  if (!msg || msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "initialpose_frame_id does not match global_frame_id");
    return;
  }
  if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000U) {
    RCLCPP_WARN(get_logger(), "Rejecting invalid initialpose timestamp");
    return;
  }
  const auto & position = msg->pose.pose.position;
  const auto & orientation = msg->pose.pose.orientation;
  Eigen::Quaterniond quaternion(
    orientation.w, orientation.x, orientation.y, orientation.z);
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !quaternion.coeffs().allFinite() ||
      !std::isfinite(quaternion.norm()) || quaternion.norm() < 1.0e-6) {
    RCLCPP_WARN(get_logger(), "Rejecting invalid initialpose position or quaternion");
    return;
  }
  initialpose_recieved_ = true;
  corrent_pose_with_cov_stamped_ptr_ =
    std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(*msg);
  corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation =
    tf2::toMsg(quaternion.normalized());
  // Reset the odom-delta baseline so the next odom callback rebuilds it
  // against the freshly-set initial pose instead of composing stale motion.
  last_odom_pose_valid_ = false;
  last_odom_received_time_ = -1.0;
  correction_evidence_.reset();
  auto_prediction_map_odom_.setIdentity();
  auto_last_scan_stamp_ = -1.0;
  auto_odom_fault_ = false;
  // Force the next cloudReceived to actually run NDT (bypassing the align
  // interval) so we re-establish map→odom against the new initial pose,
  // and drop any previously captured planar lock — the user-supplied pose
  // can't be trusted as ground truth for z/roll/pitch (RViz's SetInitialPose
  // always writes z=0, which is off by the body→base_link offset).
  have_good_pose_ = false;
  updateTfSnapshot(false);
  ndt_bootstrap_pending_ = true;
  initial_pose_stamp_ns_ = rclcpp::Time(msg->header.stamp).nanoseconds();
  last_align_stamp_s_ = -1.0e9;
  planar_locked_ = false;
  RCLCPP_INFO(
    get_logger(),
    "Initialpose accepted as NDT guess only; waiting for NDT alignment before publishing map->odom");

  // The cached scan may precede the manual pose reset by several seconds.
  // Wait for the next incoming scan instead of reusing an old successful one.
  last_scan_ptr_.reset();

  RCLCPP_INFO(get_logger(), "initialPoseReceived end");
}

void PCLLocalization::mapReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "mapReceived");
  pcl::PCLPointCloud2 raw_map_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr;

  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "map_frame_id does not match　global_frame_id");
    return;
  }

  pcl_conversions::toPCL(*msg, raw_map_cloud);
  map_cloud_ptr = to_xyzi_cloud(raw_map_cloud, get_logger(), "incoming map topic");

  if (!configureRegistrationTarget(map_cloud_ptr)) {
    RCLCPP_ERROR(get_logger(), "Rejected incoming map: incompatible registration target");
    return;
  }

  map_recieved_ = true;
  correction_evidence_.reset();
  RCLCPP_INFO(get_logger(), "mapReceived end");
}

void PCLLocalization::odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (!use_odom_ || !msg) {return;}
  // In auto mode scan-time TF supplies motion. Do not integrate later odom
  // callbacks into a scan-stamped corrected pose a second time.
  if (auto_correction_enabled_ && (have_good_pose_ || auto_odom_fault_)) {return;}
  if (msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000U ||
      (initial_pose_stamp_ns_ > 0 &&
       rclcpp::Time(msg->header.stamp).nanoseconds() < initial_pose_stamp_ns_)) {
    return;
  }

  double current_odom_received_time = msg->header.stamp.sec +
    msg->header.stamp.nanosec * 1e-9;

  // corrent_pose_with_cov_stamped_ptr_ is null until /initialpose arrives;
  // propagating motion below would SIGSEGV. Keep state fresh so we don't
  // later warn about a spurious >1 s gap or miss the first real delta.
  if (!initialpose_recieved_ || !last_odom_pose_valid_) {
    last_odom_received_time_ = current_odom_received_time;
    last_odom_pose_ = msg->pose.pose;
    last_odom_pose_valid_ = true;
    return;
  }

  double dt_odom = current_odom_received_time - last_odom_received_time_;
  if (dt_odom < 0.0 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "odom time interval is negative");
    return;
  }
  last_odom_received_time_ = current_odom_received_time;
  if (dt_odom > 1.0 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "odom time interval is too large");
    last_odom_pose_ = msg->pose.pose;
    last_odom_pose_valid_ = true;
    return;
  }
  // FAST_LIO leaves twist empty, so the previous twist-based integration
  // was a no-op. Integrate the odom-frame pose delta instead: take the
  // body-frame motion between consecutive /odom messages and compose it
  // onto the current map-frame estimate.
  Eigen::Affine3d odom_prev, odom_curr, map_to_base;
  tf2::fromMsg(last_odom_pose_, odom_prev);
  tf2::fromMsg(msg->pose.pose, odom_curr);
  tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose, map_to_base);

  Eigen::Affine3d body_delta = odom_prev.inverse() * odom_curr;
  Eigen::Affine3d new_map_to_base = map_to_base * body_delta;

  corrent_pose_with_cov_stamped_ptr_->pose.pose = tf2::toMsg(new_map_to_base);

  last_odom_pose_ = msg->pose.pose;
}

void PCLLocalization::imuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (!use_imu_) {return;}

  sensor_msgs::msg::Imu tf_converted_imu;

  try {
    const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
     base_frame_id_, msg->header.frame_id, tf2::TimePointZero);

    geometry_msgs::msg::Vector3Stamped angular_velocity, linear_acceleration, transformed_angular_velocity, transformed_linear_acceleration;
    geometry_msgs::msg::Quaternion  transformed_quaternion;

    angular_velocity.header = msg->header;
    angular_velocity.vector = msg->angular_velocity;
    linear_acceleration.header = msg->header;
    linear_acceleration.vector = msg->linear_acceleration;

    tf2::doTransform(angular_velocity, transformed_angular_velocity, transform);
    tf2::doTransform(linear_acceleration, transformed_linear_acceleration, transform);

    tf_converted_imu.angular_velocity = transformed_angular_velocity.vector;
    tf_converted_imu.linear_acceleration = transformed_linear_acceleration.vector;
    tf_converted_imu.orientation = transformed_quaternion;

  }
  catch (tf2::TransformException& ex)
  {
    std::cout << "Failed to lookup transform" << std::endl;
    RCLCPP_WARN(this->get_logger(), "Failed to lookup transform.");
    return;
  }

  Eigen::Vector3f angular_velo{tf_converted_imu.angular_velocity.x, tf_converted_imu.angular_velocity.y,
    tf_converted_imu.angular_velocity.z};
  Eigen::Vector3f acc{tf_converted_imu.linear_acceleration.x, tf_converted_imu.linear_acceleration.y, tf_converted_imu.linear_acceleration.z};
  Eigen::Quaternionf quat{msg->orientation.w, msg->orientation.x, msg->orientation.y,
    msg->orientation.z};
  double imu_time = msg->header.stamp.sec +
    msg->header.stamp.nanosec * 1e-9;

  lidar_undistortion_.getImu(angular_velo, acc, quat, imu_time);

}

void PCLLocalization::cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!localization_active_) {return;}
  // Architecture note (2026-04-20 rewrite):
  //   FAST_LIO runs continuously as odom→body at 20 Hz and is accurate to
  //   sub-decimeter over short time windows. NDT's only real job here is
  //   the one-shot map→odom offset plus slow drift correction. Running NDT
  //   every scan made it a second primary localizer that fought FAST_LIO
  //   and jumped whenever it failed.
  //
  //   New flow:
  //   1. Every cloudReceived broadcasts last_good_map_to_odom_ with node time,
  //      so TF never ages out even when the scan stamp lags during heavy NDT.
  //   2. /initialpose is only an NDT init_guess. map→odom and /pcl_pose stay
  //      unpublished until NDT passes convergence, fitness, and jump gates.
  //   3. NDT re-align only runs when ndt_align_interval_s_ has elapsed since
  //      the last attempt, including rejected bootstrap attempts.
  //   4. Any NDT result that fails converge/fitness/jump gates is
  //      discarded; we keep the last good TF.

  if (!msg) {
    RCLCPP_WARN(get_logger(), "Received null point cloud message");
    return;
  }

  // A scan queued before the pose reset cannot represent the new guess.
  // Zero-stamped initial poses retain the conventional "latest" behavior.
  if (initial_pose_stamp_ns_ > 0 &&
      rclcpp::Time(msg->header.stamp).nanoseconds() < initial_pose_stamp_ns_) {
    return;
  }
  last_scan_ptr_ = msg;
  if (!map_recieved_ || !initialpose_recieved_) {return;}

  auto broadcast_last_good = [&]() {
    rebroadcastMapOdomTF();
  };

  auto rejected_tf_status = [&]() {
    return have_good_pose_
      ? "keeping last good TF"
      : "waiting for valid NDT before publishing map->odom";
  };

  double now_s = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  if (auto_correction_enabled_ && (auto_odom_fault_ || now_s <= last_align_stamp_s_)) {
    correction_evidence_.reset();
    return;
  }
  bool should_align = (ndt_align_interval_s_ <= 0.0) ||
                      ((now_s - last_align_stamp_s_) >= ndt_align_interval_s_);

  if (!should_align) {
    broadcast_last_good();
    return;
  }

  // --- NDT re-align branch ---
  if (!auto_correction_enabled_) {last_align_stamp_s_ = now_s;}
  const auto previous_evidence = correction_evidence_;
  correction_evidence_.reset();  // Every failed attempt breaks consecutive evidence.

  geometry_msgs::msg::TransformStamped odom_to_base_link_msg;
  Eigen::Affine3d odom_at_scan = Eigen::Affine3d::Identity();
  auto odom_fault = [&]() {
    auto_odom_fault_ = true;
    have_good_pose_ = false;
    updateTfSnapshot(false);
    correction_evidence_.reset();
    RCLCPP_ERROR(get_logger(), "NDT auto correction: odom discontinuity; stopping map->odom "
      "publication. Restore healthy odometry then provide a new initialpose.");
  };
  if (auto_correction_enabled_) {
    try {
      odom_to_base_link_msg = tfbuffer_.lookupTransform(
        odom_frame_id_, base_frame_id_, msg->header.stamp,
        rclcpp::Duration::from_seconds(scan_odom_tf_timeout_));
      odom_at_scan = tf2::transformToEigen(odom_to_base_link_msg.transform);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "NDT auto correction: scan-time odom unavailable: %s", ex.what());
      broadcast_last_good();
      return;
    }
    if (!odom_at_scan.matrix().allFinite()) {
      odom_fault();
      return;
    }
    if (auto_last_scan_stamp_ >= 0.0) {
      const double dt = now_s - auto_last_scan_stamp_;
      if (dt > auto_correction_policy_.max_gap) {
        // Do not treat a long observation gap as evidence of a velocity jump.
        // Re-establish an odom baseline, but do not accept this candidate.
        auto_last_scan_odom_ = odom_at_scan;
        auto_last_scan_stamp_ = now_s;
        RCLCPP_WARN(get_logger(), "NDT auto correction: odom observation gap %.2fs; "
          "waiting for consecutive scans", dt);
        broadcast_last_good();
        return;
      }
      if (!lidar_localization::plausibleOdom(
        auto_last_scan_odom_, odom_at_scan, dt, auto_correction_policy_))
      {
        odom_fault();
        return;
      }
    }
    auto_last_scan_odom_ = odom_at_scan;
    auto_last_scan_stamp_ = now_s;
  }
  // A temporarily missing odom sample must not consume the two-second NDT
  // interval; retry the next scan as soon as exact-time TF becomes available.
  last_align_stamp_s_ = now_s;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud_ptr);

  if (msg->header.frame_id != base_frame_id_) {
    geometry_msgs::msg::TransformStamped base_to_lidar_stamped;
    try {
      base_to_lidar_stamped = tfbuffer_.lookupTransform(
          base_frame_id_, msg->header.frame_id, msg->header.stamp,
          rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
          this->get_logger(), "Could not transform %s to %s: %s",
          msg->header.frame_id.c_str(), base_frame_id_.c_str(), ex.what());
      broadcast_last_good();
      return;
    }
    Eigen::Matrix4f initial_transformation =
      tf2::transformToEigen(base_to_lidar_stamped.transform).matrix().cast<float>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*cloud_ptr, *transformed_cloud, initial_transformation);
    cloud_ptr = transformed_cloud;
  }

  if (use_imu_) {
    double received_time = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
    lidar_undistortion_.adjustDistortion(cloud_ptr, received_time);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  voxel_grid_filter_.setInputCloud(cloud_ptr);
  voxel_grid_filter_.filter(*filtered_cloud_ptr);

  pcl::PointCloud<pcl::PointXYZI> tmp;
  pcl::PointCloud<pcl::PointXYZI> height_bonus;
  pcl::PointCloud<pcl::PointXYZI> scoring_cloud;
  std::vector<int> baseline_weights;
  std::vector<int> weighted_weights;
  size_t high_points = 0;
  const int far_boost_extra_copies = std::max(far_point_boost_factor_, 1) - 1;
  for (const auto & p : filtered_cloud_ptr->points) {
    double r = std::sqrt(p.x * p.x + p.y * p.y);
    if (scan_min_range_ < r && r < scan_max_range_) {
      const int baseline_copies =
        r >= far_point_boost_min_range_ ? 1 + far_boost_extra_copies : 1;
      int weighted_copies = baseline_copies;
      tmp.push_back(p);
      if (far_boost_extra_copies > 0 && r >= far_point_boost_min_range_) {
        for (int i = 0; i < far_boost_extra_copies; ++i) {
          tmp.push_back(p);
        }
      }
      // Classification is in the base frame, before alignment. Append copies
      // AFTER voxel filtering so they survive and weight NDT's derivatives.
      if (height_boost_enabled_ && p.z >= height_boost_min_z_) {
        ++high_points;
        if (baseline_copies < 3) {
          height_bonus.push_back(p);
          ++weighted_copies;
        }
      }
      scoring_cloud.push_back(p);
      baseline_weights.push_back(baseline_copies);
      weighted_weights.push_back(weighted_copies);
    }
  }
  const size_t baseline_points = tmp.size();
  if (!height_bonus.empty()) {
    tmp += height_bonus;
  }
  if (tmp.empty()) {
    RCLCPP_WARN(get_logger(), "No source points remain after NDT range filtering, %s",
      rejected_tf_status());
    broadcast_last_good();
    return;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>(tmp));
  registration_->setInputSource(tmp_ptr);
  RCLCPP_INFO(
    get_logger(),
    "NDT source cloud: raw=%zu filtered=%zu kept=%zu frame=%s",
    cloud_ptr->size(), filtered_cloud_ptr->size(), tmp_ptr->size(),
    base_frame_id_.c_str());
  if (height_boost_enabled_) {
    RCLCPP_INFO(
      get_logger(), "NDT height weighting: frame=%s min_z=%.3f baseline=%zu "
      "high_unique=%zu added=%zu weighted=%zu",
      base_frame_id_.c_str(), height_boost_min_z_, baseline_points,
      high_points, height_bonus.size(), tmp_ptr->size());
  }

  Eigen::Affine3d init_affine;
  tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose, init_affine);
  Eigen::Matrix4f init_guess = init_affine.matrix().cast<float>();
  if (auto_correction_enabled_ && have_good_pose_ && !ndt_bootstrap_pending_) {
    init_guess = (auto_prediction_map_odom_ * odom_at_scan).matrix().cast<float>();
  }

  // Clip init_guess onto the locked plane only if we've already captured
  // the lock values from a prior successful NDT match. On the very first
  // align after /initialpose we let NDT explore full 6DOF, because the
  // user-supplied pose has z=0 (RViz convention) while the true z offset
  // to the body-frame map origin is ~0.12 m (URDF imu_joint z).
  if (lock_planar_ && planar_locked_) {
    Eigen::Matrix3f R = init_guess.block<3, 3>(0, 0);
    double yaw = std::atan2(R(1, 0), R(0, 0));
    init_guess.block<3, 3>(0, 0) = (
      Eigen::AngleAxisf(static_cast<float>(locked_roll_), Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(static_cast<float>(locked_pitch_), Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(static_cast<float>(yaw), Eigen::Vector3f::UnitZ())
    ).toRotationMatrix();
    init_guess(2, 3) = static_cast<float>(locked_z_);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  const auto align_start = std::chrono::steady_clock::now();
  registration_->align(*output_cloud, init_guess);
  const double align_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - align_start).count();

  const bool has_converged = registration_->hasConverged();
  Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();

  if (!has_converged) {
    RCLCPP_WARN(get_logger(), "NDT didn't converge, %s", rejected_tf_status());
    broadcast_last_good();
    return;
  }
  if (!final_transformation.allFinite()) {
    RCLCPP_WARN(get_logger(), "NDT returned a non-finite transform, %s", rejected_tf_status());
    broadcast_last_good();
    return;
  }
  const auto fitness_start = std::chrono::steady_clock::now();
  const auto scores = lidar_localization::evaluateWeightedFitness(
    scoring_cloud, baseline_weights, weighted_weights, final_transformation,
    *registration_->getSearchMethodTarget());
  const double weighted_fitness = scores.weighted;
  double fitness_score = scores.baseline;
  if (height_boost_enabled_) {
    const double fitness_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - fitness_start).count();
    RCLCPP_INFO(
      get_logger(), "NDT height fitness: baseline=%.6f weighted=%.6f "
      "align_ms=%.1f fitness_ms=%.1f nn_queries=%zu (gate=baseline)",
      fitness_score, weighted_fitness, align_ms, fitness_ms, scores.searches);
  }
  if (!std::isfinite(fitness_score) || fitness_score > score_threshold_) {
    RCLCPP_WARN(
      get_logger(),
      "Fitness %.3f > %.3f, %s; init=[%.2f, %.2f, %.2f, yaw=%.3f] "
      "result=[%.2f, %.2f, %.2f, yaw=%.3f]",
      fitness_score, score_threshold_, rejected_tf_status(),
      init_guess(0, 3), init_guess(1, 3), init_guess(2, 3), yaw_from_matrix(init_guess),
      final_transformation(0, 3), final_transformation(1, 3), final_transformation(2, 3),
      yaw_from_matrix(final_transformation));
    broadcast_last_good();
    return;
  }

  // A manual re-init may move arbitrarily far from the previous solution,
  // but the NDT result must stay near the NEW user guess. Bootstrap limits
  // are separate from the stricter steady-state tracking limits.
  if (ndt_bootstrap_pending_) {
    const Eigen::Matrix4f delta = init_guess.inverse() * final_transformation;
    const double dtrans = delta.block<3, 1>(0, 3).norm();
    const double rotation = Eigen::AngleAxisf(delta.block<3, 3>(0, 0)).angle();
    const double translation_limit =
      max_bootstrap_pose_jump_translation_ > 0.0
      ? max_bootstrap_pose_jump_translation_ : max_pose_jump_translation_;
    const double rotation_limit =
      max_bootstrap_pose_jump_rotation_ > 0.0
      ? max_bootstrap_pose_jump_rotation_ : max_pose_jump_rotation_;
    if ((translation_limit > 0.0 && dtrans > translation_limit) ||
        (rotation_limit > 0.0 && rotation > rotation_limit)) {
      RCLCPP_WARN(
        get_logger(),
        "NDT jump rejected: bootstrap=%d translation=%.3f/%.3f rotation=%.3f/%.3f; %s",
        ndt_bootstrap_pending_, dtrans, translation_limit, rotation, rotation_limit,
        rejected_tf_status());
      broadcast_last_good();
      return;
    }
  }

  // Compute map→odom = map→base_link * (odom→base_link)^-1. Do this before
  // mutating corrent_pose_with_cov_stamped_ptr_; a candidate that cannot
  // produce a sane map→odom must not poison the next init_guess.
  try {
    if (!auto_correction_enabled_) {
      odom_to_base_link_msg = tfbuffer_.lookupTransform(
        odom_frame_id_, base_frame_id_, msg->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "odom→base_link lookup failed: %s", ex.what());
    broadcast_last_good();
    return;
  }
  if (auto_correction_enabled_ && have_good_pose_ && !ndt_bootstrap_pending_ &&
    !lidar_localization::withinCorrection(
      Eigen::Affine3d(init_guess.cast<double>()),
      Eigen::Affine3d(final_transformation.cast<double>()),
      auto_correction_policy_.max_translation, auto_correction_policy_.max_rotation))
  {
    RCLCPP_WARN(get_logger(), "NDT auto correction: raw candidate exceeds absolute 3D bounds");
    broadcast_last_good();
    return;
  }
  // First good match — freeze z/roll/pitch from NDT's own estimate. Avoids
  // the "user initialpose z=0 vs true z≈0.12" bias that otherwise poisons
  // the lock.
  if (lock_planar_ && !planar_locked_) {
    const Eigen::Matrix3f R = final_transformation.block<3, 3>(0, 0);
    // RPY from rotation matrix (Z-Y-X order); atan2 avoids eulerAngles gimbal.
    locked_roll_ = std::atan2(R(2, 1), R(2, 2));
    locked_pitch_ = std::atan2(
      -R(2, 0), std::sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
    locked_z_ = final_transformation(2, 3);
    planar_locked_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Planar lock captured from first NDT: z=%.3f roll=%.3f pitch=%.3f",
      locked_z_, locked_roll_, locked_pitch_);
  }

  if (lock_planar_ && planar_locked_) {
    Eigen::Matrix3f R = final_transformation.block<3, 3>(0, 0);
    double yaw = std::atan2(R(1, 0), R(0, 0));
    final_transformation.block<3, 3>(0, 0) = (
      Eigen::AngleAxisf(static_cast<float>(locked_roll_), Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(static_cast<float>(locked_pitch_), Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(static_cast<float>(yaw), Eigen::Vector3f::UnitZ())
    ).toRotationMatrix();
    final_transformation(2, 3) = static_cast<float>(locked_z_);
    if (auto_correction_enabled_) {
      fitness_score = lidar_localization::evaluateWeightedFitness(
        scoring_cloud, baseline_weights, weighted_weights, final_transformation,
        *registration_->getSearchMethodTarget()).baseline;
      if (!std::isfinite(fitness_score) || fitness_score > score_threshold_) {
        RCLCPP_WARN(get_logger(), "NDT auto correction: projected pose fails baseline fitness");
        broadcast_last_good();
        return;
      }
    }
  }

  // Preserve tracking semantics: an established planar lock is applied before
  // checking the steady-state jump, unlike the raw 6DOF bootstrap gate above.
  if (!auto_correction_enabled_ && have_good_pose_ && !ndt_bootstrap_pending_) {
    const Eigen::Matrix4f delta = init_guess.inverse() * final_transformation;
    const double dtrans = delta.block<3, 1>(0, 3).norm();
    const double dyaw = std::abs(yaw_from_matrix(delta));
    if (dtrans > max_pose_jump_translation_ || dyaw > max_pose_jump_rotation_) {
      RCLCPP_WARN(get_logger(),
        "NDT jump (trans=%.3f m, yaw=%.3f rad), keeping last good TF", dtrans, dyaw);
      broadcast_last_good();
      return;
    }
  }

  Eigen::Affine3d odom_to_base_affine = tf2::transformToEigen(odom_to_base_link_msg.transform);
  Eigen::Affine3d map_to_base_affine(final_transformation.cast<double>());
  Eigen::Affine3d map_to_odom_new = map_to_base_affine * odom_to_base_affine.inverse();

  if (!auto_correction_enabled_ && have_good_pose_ && !ndt_bootstrap_pending_ &&
      (max_map_odom_update_translation_ > 0.0 ||
       max_map_odom_update_rotation_ > 0.0)) {
    Eigen::Affine3d map_to_odom_old(last_good_map_to_odom_.cast<double>());
    Eigen::Affine3d map_odom_delta = map_to_odom_old.inverse() * map_to_odom_new;
    const Eigen::Vector3d dt = map_odom_delta.translation();
    const double dtrans = std::sqrt(dt.x() * dt.x() + dt.y() * dt.y() + dt.z() * dt.z());
    const Eigen::Matrix3d dR = map_odom_delta.rotation();
    const double dyaw = std::atan2(dR(1, 0), dR(0, 0));
    const bool trans_bad =
      max_map_odom_update_translation_ > 0.0 &&
      dtrans > max_map_odom_update_translation_;
    const bool yaw_bad =
      max_map_odom_update_rotation_ > 0.0 &&
      std::abs(dyaw) > max_map_odom_update_rotation_;
    if (trans_bad || yaw_bad) {
      RCLCPP_WARN(
        get_logger(),
        "NDT map→odom jump (trans=%.3f m, yaw=%.3f rad), keeping last good TF",
        dtrans, dyaw);
      broadcast_last_good();
      return;
    }
  }

  // Smooth against the previous good map→odom using an exponential blend.
  // Linear interpolation on translation + SLERP on rotation so the TF
  // downstream sees small, continuous corrections instead of occasional
  // instantaneous jumps from a slightly misaligned NDT lock.
  Eigen::Affine3d map_to_odom_final;
  if (auto_correction_enabled_ && have_good_pose_ && !ndt_bootstrap_pending_) {
    const Eigen::Affine3d prediction(init_guess.cast<double>());
    const Eigen::Affine3d published(last_good_map_to_odom_.cast<double>());
    if (!lidar_localization::withinCorrection(prediction, map_to_base_affine,
      auto_correction_policy_.max_translation, auto_correction_policy_.max_rotation) ||
      !lidar_localization::withinCorrection(published * odom_at_scan, map_to_base_affine,
      auto_correction_policy_.max_translation, auto_correction_policy_.max_rotation))
    {
      RCLCPP_WARN(get_logger(), "NDT auto correction: correction outside recovery range");
      broadcast_last_good();
      return;
    }
    const bool large_correction = !lidar_localization::withinCorrection(
      prediction, map_to_base_affine, max_pose_jump_translation_, max_pose_jump_rotation_);
    if (large_correction) {
      const double initial_fitness = lidar_localization::evaluateWeightedFitness(
        scoring_cloud, baseline_weights, weighted_weights, init_guess,
        *registration_->getSearchMethodTarget()).baseline;
      const double improvement = initial_fitness - fitness_score;
      if (!std::isfinite(initial_fitness) || improvement <= 1.0e-9 ||
        improvement < auto_correction_policy_.min_improvement * initial_fitness)
      {
        RCLCPP_WARN(get_logger(), "NDT auto correction: insufficient improvement "
          "initial=%.6f candidate=%.6f", initial_fitness, fitness_score);
        broadcast_last_good();
        return;
      }
      correction_evidence_ = previous_evidence;
      const bool confirmed = correction_evidence_.confirm(
        map_to_odom_new, odom_at_scan, now_s, auto_correction_policy_);
      RCLCPP_INFO(get_logger(), "NDT auto correction: evidence=%d/%d "
        "initial=%.6f candidate=%.6f translation=%.3f rotation=%.3f",
        correction_evidence_.count, auto_correction_policy_.confirmations,
        initial_fitness, fitness_score,
        (map_to_base_affine.translation() - prediction.translation()).norm(),
        lidar_localization::rotationDistance(prediction, map_to_base_affine));
      if (!confirmed) {
        broadcast_last_good();
        return;
      }
    }
    // NDT may take seconds. Limit the controller-visible jump at the latest
    // robot position, not at the old scan pose or the distant odom origin.
    Eigen::Affine3d odom_at_application;
    try {
      const auto latest = tfbuffer_.lookupTransform(
        odom_frame_id_, base_frame_id_, tf2::TimePointZero);
      odom_at_application = tf2::transformToEigen(latest.transform);
      double latest_stamp = latest.header.stamp.sec + latest.header.stamp.nanosec * 1e-9;
      if (latest_stamp == 0.0) {latest_stamp = now_s;}  // Static TF, used by offline tests.
      const double application_delay = latest_stamp - now_s;
      if (application_delay < 0.0 || application_delay > auto_correction_policy_.max_gap) {
        correction_evidence_.reset();
        RCLCPP_WARN(get_logger(), "NDT auto correction: application odom time gap %.2fs; "
          "discarding stale candidate", application_delay);
        broadcast_last_good();
        return;
      }
      if (!lidar_localization::plausibleOdom(odom_at_scan, odom_at_application,
        application_delay, auto_correction_policy_))
      {
        odom_fault();
        return;
      }
    } catch (tf2::TransformException & ex) {
      correction_evidence_.reset();
      RCLCPP_WARN(get_logger(), "NDT auto correction: latest odom unavailable: %s", ex.what());
      broadcast_last_good();
      return;
    }
    if (!lidar_localization::withinCorrection(
      published * odom_at_application, map_to_odom_new * odom_at_application,
      auto_correction_policy_.max_translation, auto_correction_policy_.max_rotation))
    {
      correction_evidence_.reset();
      RCLCPP_WARN(get_logger(), "NDT auto correction: current-pose correction outside recovery range");
      broadcast_last_good();
      return;
    }
    map_to_odom_final = lidar_localization::boundedCorrection(
      published, map_to_odom_new, odom_at_application, map_odom_smoothing_,
      auto_correction_policy_);
    const Eigen::Affine3d before = published * odom_at_application;
    const Eigen::Affine3d applied = map_to_odom_final * odom_at_application;
    const Eigen::Affine3d target = map_to_odom_new * odom_at_application;
    RCLCPP_INFO(get_logger(), "NDT auto correction: applied translation=%.3f rotation=%.3f "
      "remaining_translation=%.3f remaining_rotation=%.3f",
      (applied.translation() - before.translation()).norm(),
      lidar_localization::rotationDistance(before, applied),
      (target.translation() - applied.translation()).norm(),
      lidar_localization::rotationDistance(applied, target));
    correction_evidence_.reset();
  } else if (!have_good_pose_ || map_odom_smoothing_ >= 1.0) {
    map_to_odom_final = map_to_odom_new;
  } else {
    Eigen::Affine3d map_to_odom_old(last_good_map_to_odom_.cast<double>());
    const double alpha = map_odom_smoothing_;
    Eigen::Vector3d t_blend =
      alpha * map_to_odom_new.translation() +
      (1.0 - alpha) * map_to_odom_old.translation();
    Eigen::Quaterniond q_old(map_to_odom_old.rotation());
    Eigen::Quaterniond q_new(map_to_odom_new.rotation());
    Eigen::Quaterniond q_blend = q_old.slerp(alpha, q_new);
    map_to_odom_final = Eigen::Affine3d::Identity();
    map_to_odom_final.translation() = t_blend;
    map_to_odom_final.linear() = q_blend.toRotationMatrix();
  }

  if (auto_correction_enabled_) {
    auto_prediction_map_odom_ = map_to_odom_new;
    // Pose/path describe the applied correction, while aligned_scan remains
    // the raw optimizer result for diagnosis. The next guess uses the raw model.
    final_transformation = (map_to_odom_final * odom_at_scan).matrix().cast<float>();
  }
  last_good_map_to_odom_ = map_to_odom_final.matrix().cast<float>();
  have_good_pose_ = true;
  updateTfSnapshot(true);
  ndt_bootstrap_pending_ = false;
  last_align_stamp_s_ = now_s;

  Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
  Eigen::Quaterniond quat_eig(rot_mat);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);
  corrent_pose_with_cov_stamped_ptr_->header.stamp = msg->header.stamp;
  corrent_pose_with_cov_stamped_ptr_->header.frame_id = global_frame_id_;
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x = final_transformation(0, 3);
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y = final_transformation(1, 3);
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z = final_transformation(2, 3);
  corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = quat_msg;

  if (!enable_timer_publishing_) {
    pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);
  }

  sensor_msgs::msg::PointCloud2 aligned_scan_msg;
  pcl::toROSMsg(*output_cloud, aligned_scan_msg);
  aligned_scan_msg.header.stamp = now();
  aligned_scan_msg.header.frame_id = global_frame_id_;
  ndt_aligned_scan_pub_->publish(aligned_scan_msg);

  const Eigen::Matrix3f map_odom_rotation = last_good_map_to_odom_.block<3, 3>(0, 0);
  const double map_odom_yaw = std::atan2(map_odom_rotation(1, 0), map_odom_rotation(0, 0));

  RCLCPP_INFO(
    get_logger(),
    "NDT aligned: fitness=%.3f map→odom=[%.2f, %.2f, %.2f, yaw=%.3f]",
    fitness_score,
    last_good_map_to_odom_(0, 3),
    last_good_map_to_odom_(1, 3),
    last_good_map_to_odom_(2, 3),
    map_odom_yaw);

  broadcast_last_good();

  if (!enable_timer_publishing_) {
    geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped_ptr(new geometry_msgs::msg::PoseStamped);
    pose_stamped_ptr->header.stamp = msg->header.stamp;
    pose_stamped_ptr->header.frame_id = global_frame_id_;
    pose_stamped_ptr->pose = corrent_pose_with_cov_stamped_ptr_->pose.pose;
    path_ptr_->poses.push_back(*pose_stamped_ptr);
    path_pub_->publish(*path_ptr_);
  }

  if (enable_debug_) {
    std::cout << "NDT filtered_points=" << filtered_cloud_ptr->size()
              << " fitness=" << fitness_score
              << " has_converged=" << has_converged << std::endl;
    std::cout << "final_transformation:\n" << final_transformation << std::endl;
  }
}

void PCLLocalization::timerPublishPose()
{
  if (!localization_active_ || !corrent_pose_with_cov_stamped_ptr_ || !have_good_pose_) {return;}
  geometry_msgs::msg::PoseWithCovarianceStamped pose_copy = *corrent_pose_with_cov_stamped_ptr_;
  pose_copy.header.stamp = now();

  geometry_msgs::msg::PoseStamped stamped;
  stamped.header = pose_copy.header;
  stamped.header.frame_id = global_frame_id_;
  stamped.pose = pose_copy.pose.pose;
  path_ptr_->poses.push_back(stamped);

  nav_msgs::msg::Path path_copy = *path_ptr_;

  pose_pub_->publish(pose_copy);
  path_pub_->publish(path_copy);

  geometry_msgs::msg::TransformStamped map_to_base_link_stamped;
  map_to_base_link_stamped.header.stamp = pose_copy.header.stamp;
  map_to_base_link_stamped.header.frame_id = global_frame_id_;
  map_to_base_link_stamped.child_frame_id = base_frame_id_;
  map_to_base_link_stamped.transform.translation.x = pose_copy.pose.pose.position.x;
  map_to_base_link_stamped.transform.translation.y = pose_copy.pose.pose.position.y;
  map_to_base_link_stamped.transform.translation.z = pose_copy.pose.pose.position.z;
  map_to_base_link_stamped.transform.rotation = pose_copy.pose.pose.orientation;

  if (!enable_map_odom_tf_) {
    broadcaster_.sendTransform(map_to_base_link_stamped);
  } else {
    tf2::Transform map_to_base_link_tf;
    tf2::fromMsg(map_to_base_link_stamped.transform, map_to_base_link_tf);

    geometry_msgs::msg::TransformStamped odom_to_base_link_msg;
    try {
      odom_to_base_link_msg = tfbuffer_.lookupTransform(
        odom_frame_id_, base_frame_id_, pose_copy.header.stamp, rclcpp::Duration::from_seconds(0.1));
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(), "Could not get transform %s to %s: %s",
        base_frame_id_.c_str(), odom_frame_id_.c_str(), ex.what());
      return;
    }
    tf2::Transform odom_to_base_link_tf;
    tf2::fromMsg(odom_to_base_link_msg.transform, odom_to_base_link_tf);

    tf2::Transform map_to_odom_tf = map_to_base_link_tf * odom_to_base_link_tf.inverse();
    geometry_msgs::msg::TransformStamped map_to_odom_stamped;
    map_to_odom_stamped.header.stamp = pose_copy.header.stamp;
    map_to_odom_stamped.header.frame_id = global_frame_id_;
    map_to_odom_stamped.child_frame_id = odom_frame_id_;
    map_to_odom_stamped.transform = tf2::toMsg(map_to_odom_tf);
    
    broadcaster_.sendTransform(map_to_odom_stamped);
  }
}
