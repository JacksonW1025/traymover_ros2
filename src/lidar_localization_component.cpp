#include <lidar_localization/lidar_localization_component.hpp>
#include <lidar_localization/ndt_map_compatibility.hpp>
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
  declare_parameter("scan_max_range", 100.0);
  declare_parameter("scan_min_range", 1.0);
  declare_parameter("far_point_boost_min_range", 0.0);
  declare_parameter("far_point_boost_factor", 1);
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
  declare_parameter("ndt_align_interval_s", 5.0);
  declare_parameter("tf_rebroadcast_rate_hz", 50.0);
  declare_parameter("map_odom_smoothing", 0.3);
  declare_parameter("max_map_odom_update_translation", -1.0);
  declare_parameter("max_map_odom_update_rotation", -1.0);
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn PCLLocalization::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  initializeParameters();
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

    RCLCPP_INFO(get_logger(), "Map Size %ld", map_cloud_ptr->size());
    log_cloud_bounds(get_logger(), "NDT target map", *map_cloud_ptr);
    sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*map_cloud_ptr, *map_msg_ptr);
    map_msg_ptr->header.frame_id = global_frame_id_;
    initial_map_pub_->publish(*map_msg_ptr);
    RCLCPP_INFO(get_logger(), "Initial Map Published");

    if (!configureRegistrationTarget(map_cloud_ptr)) {
      return CallbackReturn::FAILURE;
    }

    map_recieved_ = true;
  }

  // High-frequency TF rebroadcaster. Keeps map→odom fresh independent of
  // LiDAR scan rate, so downstream consumers (RViz RobotModel, Nav2
  // costmaps) see the latest FAST_LIO pose through a fresh TF chain.
  if (tf_rebroadcast_rate_hz_ > 0.0) {
    const auto period = std::chrono::duration<double>(1.0 / tf_rebroadcast_rate_hz_);
    tf_rebroadcast_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PCLLocalization::rebroadcastMapOdomTF, this));
  }

  RCLCPP_INFO(get_logger(), "Activating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

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

void PCLLocalization::rebroadcastMapOdomTF()
{
  if (!have_good_pose_) return;
  Eigen::Affine3d affine(last_good_map_to_odom_.cast<double>());
  geometry_msgs::msg::TransformStamped tf_msg = tf2::eigenToTransform(affine);
  tf_msg.header.stamp = now();
  tf_msg.header.frame_id = global_frame_id_;
  tf_msg.child_frame_id = odom_frame_id_;
  broadcaster_.sendTransform(tf_msg);
}

CallbackReturn PCLLocalization::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning Up");
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

  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_FATAL(get_logger(), "Error Processing from %s", state.label().c_str());

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
  get_parameter("scan_max_range", scan_max_range_);
  get_parameter("scan_min_range", scan_min_range_);
  get_parameter("far_point_boost_min_range", far_point_boost_min_range_);
  get_parameter("far_point_boost_factor", far_point_boost_factor_);
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
  get_parameter("ndt_align_interval_s", ndt_align_interval_s_);
  get_parameter("tf_rebroadcast_rate_hz", tf_rebroadcast_rate_hz_);
  get_parameter("map_odom_smoothing", map_odom_smoothing_);
  get_parameter("max_map_odom_update_translation", max_map_odom_update_translation_);
  get_parameter("max_map_odom_update_rotation", max_map_odom_update_rotation_);
  RCLCPP_INFO(get_logger(), "lock_planar: %d", lock_planar_);
  RCLCPP_INFO(get_logger(), "max_pose_jump_translation: %.2f m", max_pose_jump_translation_);
  RCLCPP_INFO(get_logger(), "max_pose_jump_rotation: %.2f rad", max_pose_jump_rotation_);
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
  const pcl::PointCloud<pcl::PointXYZI>::Ptr & map_cloud_ptr)
{
  if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>());
    voxel_grid_filter_.setInputCloud(map_cloud_ptr);
    voxel_grid_filter_.filter(*filtered_cloud_ptr);
    registration_->setInputTarget(filtered_cloud_ptr);
    return true;
  }

  if (!configureNdtResolutionForMap(*map_cloud_ptr)) {
    return false;
  }
  registration_->setInputTarget(map_cloud_ptr);
  return true;
}

void PCLLocalization::initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "initialPoseReceived");
  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "initialpose_frame_id does not match global_frame_id");
    return;
  }
  initialpose_recieved_ = true;
  corrent_pose_with_cov_stamped_ptr_ = msg;
  // Reset the odom-delta baseline so the next odom callback rebuilds it
  // against the freshly-set initial pose instead of composing stale motion.
  last_odom_pose_valid_ = false;
  // Force the next cloudReceived to actually run NDT (bypassing the align
  // interval) so we re-establish map→odom against the new initial pose,
  // and drop any previously captured planar lock — the user-supplied pose
  // can't be trusted as ground truth for z/roll/pitch (RViz's SetInitialPose
  // always writes z=0, which is off by the body→base_link offset).
  have_good_pose_ = false;
  ndt_bootstrap_pending_ = true;
  last_align_stamp_s_ = -1.0e9;
  planar_locked_ = false;
  RCLCPP_INFO(
    get_logger(),
    "Initialpose accepted as NDT guess only; waiting for NDT alignment before publishing map->odom");

  if(last_scan_ptr_) {
    cloudReceived(last_scan_ptr_);
  }

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
  RCLCPP_INFO(get_logger(), "mapReceived end");
}

void PCLLocalization::odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (!use_odom_) {return;}

  double current_odom_received_time = msg->header.stamp.sec +
    msg->header.stamp.nanosec * 1e-9;

  // corrent_pose_with_cov_stamped_ptr_ is null until /initialpose arrives;
  // propagating motion below would SIGSEGV. Keep state fresh so we don't
  // later warn about a spurious >1 s gap or miss the first real delta.
  if (!initialpose_recieved_) {
    last_odom_received_time_ = current_odom_received_time;
    last_odom_pose_ = msg->pose.pose;
    last_odom_pose_valid_ = true;
    return;
  }

  double dt_odom = current_odom_received_time - last_odom_received_time_;
  last_odom_received_time_ = current_odom_received_time;
  if (dt_odom > 1.0 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "odom time interval is too large");
    last_odom_pose_ = msg->pose.pose;
    last_odom_pose_valid_ = true;
    return;
  }
  if (dt_odom < 0.0 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "odom time interval is negative");
    return;
  }

  // FAST_LIO leaves twist empty, so the previous twist-based integration
  // was a no-op. Integrate the odom-frame pose delta instead: take the
  // body-frame motion between consecutive /odom messages and compose it
  // onto the current map-frame estimate.
  if (!last_odom_pose_valid_) {
    last_odom_pose_ = msg->pose.pose;
    last_odom_pose_valid_ = true;
    return;
  }

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

  if (!map_recieved_ || !initialpose_recieved_) {return;}

  auto broadcast_last_good = [&]() {
    if (!have_good_pose_) return;
    Eigen::Affine3d last_good_affine(last_good_map_to_odom_.cast<double>());
    geometry_msgs::msg::TransformStamped tf_msg = tf2::eigenToTransform(last_good_affine);
    tf_msg.header.stamp = now();
    tf_msg.header.frame_id = global_frame_id_;
    tf_msg.child_frame_id = odom_frame_id_;
    broadcaster_.sendTransform(tf_msg);
  };

  auto rejected_tf_status = [&]() {
    return have_good_pose_
      ? "keeping last good TF"
      : "waiting for valid NDT before publishing map->odom";
  };

  double now_s = msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9;
  bool should_align = (ndt_align_interval_s_ <= 0.0) ||
                      ((now_s - last_align_stamp_s_) >= ndt_align_interval_s_);

  if (!should_align) {
    broadcast_last_good();
    return;
  }

  // --- NDT re-align branch ---
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
  const int far_boost_extra_copies = std::max(far_point_boost_factor_, 1) - 1;
  for (const auto & p : filtered_cloud_ptr->points) {
    double r = std::sqrt(p.x * p.x + p.y * p.y);
    if (scan_min_range_ < r && r < scan_max_range_) {
      tmp.push_back(p);
      if (far_boost_extra_copies > 0 && r >= far_point_boost_min_range_) {
        for (int i = 0; i < far_boost_extra_copies; ++i) {
          tmp.push_back(p);
        }
      }
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>(tmp));
  registration_->setInputSource(tmp_ptr);
  RCLCPP_INFO(
    get_logger(),
    "NDT source cloud: raw=%zu filtered=%zu kept=%zu frame=%s",
    cloud_ptr->size(), filtered_cloud_ptr->size(), tmp_ptr->size(),
    base_frame_id_.c_str());

  Eigen::Affine3d init_affine;
  tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose, init_affine);
  Eigen::Matrix4f init_guess = init_affine.matrix().cast<float>();

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
  registration_->align(*output_cloud, init_guess);

  const bool has_converged = registration_->hasConverged();
  const double fitness_score = registration_->getFitnessScore();
  Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();

  if (!has_converged) {
    RCLCPP_WARN(get_logger(), "NDT didn't converge, %s", rejected_tf_status());
    broadcast_last_good();
    return;
  }
  if (fitness_score > score_threshold_) {
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
  }

  // Jump gate (only meaningful once we have a previous good reference).
  if (have_good_pose_ && !ndt_bootstrap_pending_) {
    Eigen::Matrix4f delta = init_guess.inverse() * final_transformation;
    double dx = delta(0, 3), dy = delta(1, 3), dz = delta(2, 3);
    double dtrans = std::sqrt(dx * dx + dy * dy + dz * dz);
    Eigen::Matrix3f dR = delta.block<3, 3>(0, 0);
    double dyaw = std::atan2(dR(1, 0), dR(0, 0));
    if (dtrans > max_pose_jump_translation_ ||
        std::abs(dyaw) > max_pose_jump_rotation_) {
      RCLCPP_WARN(
        get_logger(),
        "NDT jump (trans=%.3f m, yaw=%.3f rad), keeping last good TF",
        dtrans, dyaw);
      broadcast_last_good();
      return;
    }
  }

  // Compute map→odom = map→base_link * (odom→base_link)^-1. Do this before
  // mutating corrent_pose_with_cov_stamped_ptr_; a candidate that cannot
  // produce a sane map→odom must not poison the next init_guess.
  geometry_msgs::msg::TransformStamped odom_to_base_link_msg;
  try {
    odom_to_base_link_msg = tfbuffer_.lookupTransform(
      odom_frame_id_, base_frame_id_, msg->header.stamp,
      rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(
      this->get_logger(), "odom→base_link lookup failed: %s", ex.what());
    broadcast_last_good();
    return;
  }
  Eigen::Affine3d odom_to_base_affine = tf2::transformToEigen(odom_to_base_link_msg.transform);
  Eigen::Affine3d map_to_base_affine(final_transformation.cast<double>());
  Eigen::Affine3d map_to_odom_new = map_to_base_affine * odom_to_base_affine.inverse();

  if (have_good_pose_ && !ndt_bootstrap_pending_ &&
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
  if (!have_good_pose_ || map_odom_smoothing_ >= 1.0) {
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

  last_good_map_to_odom_ = map_to_odom_final.matrix().cast<float>();
  have_good_pose_ = true;
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

  last_scan_ptr_ = msg;

  if (enable_debug_) {
    std::cout << "NDT filtered_points=" << filtered_cloud_ptr->size()
              << " fitness=" << fitness_score
              << " has_converged=" << has_converged << std::endl;
    std::cout << "final_transformation:\n" << final_transformation << std::endl;
  }
}

void PCLLocalization::timerPublishPose()
{
  if (!corrent_pose_with_cov_stamped_ptr_ || !have_good_pose_) {return;}
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
