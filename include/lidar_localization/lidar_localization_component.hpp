#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/ply_io.h>

#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
#include <pclomp/gicp_omp.h>
#include <pclomp/gicp_omp_impl.hpp>

#include "lidar_localization/lidar_undistortion.hpp"
#include "lidar_localization/ndt_auto_correction.hpp"

using namespace std::chrono_literals;

class PCLLocalization : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PCLLocalization(const rclcpp::NodeOptions & options);

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturn on_error(const rclcpp_lifecycle::State & state);

  void initializeParameters();
  void initializePubSub();
  void initializeRegistration();
  bool configureRegistrationTarget(
    pcl::PointCloud<pcl::PointXYZI>::Ptr & map_cloud_ptr);
  bool configureNdtResolutionForMap(
    const pcl::PointCloud<pcl::PointXYZI> & map_cloud);
  bool setEffectiveNdtResolution(double resolution);
  void initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void mapReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void imuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  // void gnssReceived();

  tf2_ros::TransformBroadcaster broadcaster_;
  rclcpp::Clock clock_;
  tf2_ros::Buffer tfbuffer_;
  tf2_ros::TransformListener tflistener_;

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::ConstSharedPtr
    initial_pose_sub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
    path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    initial_map_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    ndt_aligned_scan_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr
    map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr
    odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr
    cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr
    imu_sub_;

  boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> registration_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr corrent_pose_with_cov_stamped_ptr_;
  nav_msgs::msg::Path::SharedPtr path_ptr_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr last_scan_ptr_;

  bool map_recieved_{false};
  bool initialpose_recieved_{false};
  bool localization_active_{false};

  // parameters
  std::string global_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  std::string registration_method_;
  double scan_max_range_;
  double scan_min_range_;
  double far_point_boost_min_range_{0.0};
  int far_point_boost_factor_{1};
  bool height_boost_enabled_{false};
  double height_boost_min_z_{2.0};
  bool auto_correction_enabled_{false};
  lidar_localization::AutoCorrectionPolicy auto_correction_policy_;
  lidar_localization::CorrectionEvidence correction_evidence_;
  Eigen::Affine3d auto_prediction_map_odom_{Eigen::Affine3d::Identity()};
  Eigen::Affine3d auto_last_scan_odom_{Eigen::Affine3d::Identity()};
  double auto_last_scan_stamp_{-1.0};
  bool auto_odom_fault_{false};
  double scan_period_;
  double score_threshold_;
  double ndt_resolution_;
  double ndt_step_size_;
  double transform_epsilon_;
  double voxel_leaf_size_;
  double map_voxel_leaf_size_{0.0};
  bool use_pcd_map_{false};
  std::string map_path_;
  bool set_initial_pose_{false};
  double initial_pose_x_;
  double initial_pose_y_;
  double initial_pose_z_;
  double initial_pose_qx_;
  double initial_pose_qy_;
  double initial_pose_qz_;
  double initial_pose_qw_;

  bool use_odom_{false};
  double last_odom_received_time_;
  // FAST_LIO publishes nav_msgs/Odometry with an empty twist field, so
  // twist-based integration of the NDT init_guess was a no-op. Instead we
  // propagate using the pose delta between consecutive odom messages.
  geometry_msgs::msg::Pose last_odom_pose_;
  bool last_odom_pose_valid_{false};

  // Planar-lock guards: in narrow indoor spaces 6DOF NDT is under-constrained
  // on z/roll/pitch (parallel walls give no vertical-axis signal), which
  // produces tilted robot models and large pose jumps. For a ground robot we
  // can force z/roll/pitch back to the first accepted NDT match after every
  // later match, reducing the effective DOF to x/y/yaw.
  bool lock_planar_{false};
  double locked_z_{0.0};
  double locked_roll_{0.0};
  double locked_pitch_{0.0};
  bool planar_locked_{false};  // true once a valid NDT match populated the locks

  // Reject an NDT result whose xyz/yaw delta from the init_guess exceeds
  // these bounds — NDT latching onto repeating geometry (parallel walls,
  // similar columns) is the main failure mode in narrow corridors.
  double max_pose_jump_translation_{1.0};  // m
  double max_pose_jump_rotation_{0.5};     // rad
  // First match after each /initialpose is bounded against the new guess.
  // Nonpositive values fall back to the steady-state jump limits.
  double max_bootstrap_pose_jump_translation_{-1.0};
  double max_bootstrap_pose_jump_rotation_{-1.0};

  // Architecture: FAST_LIO provides continuous tight-coupled odom at 20 Hz,
  // so NDT's real job is a one-shot /map→/odom offset plus slow drift
  // correction. Running NDT every scan made it the main localization and
  // caused jumps whenever it failed. Now we only re-align every
  // ndt_align_interval_s_ seconds (scan time); between aligns we keep
  // broadcasting the last good map→odom with the current stamp so TF
  // never ages out.
  double ndt_align_interval_s_{5.0};
  double scan_odom_tf_timeout_{0.5};
  // NDT-validated map->odom only. /initialpose is an initial guess, not proof
  // of localization, so it must not set have_good_pose_.
  Eigen::Matrix4f last_good_map_to_odom_{Eigen::Matrix4f::Identity()};
  bool have_good_pose_{false};
  bool ndt_bootstrap_pending_{false};
  int64_t initial_pose_stamp_ns_{0};
  double last_align_stamp_s_{0.0};

  // Exponential smoothing weight for new NDT map→odom results.
  // new_good = slerp(old_good, ndt_result, map_odom_smoothing_).
  // 1.0 = no smoothing (old behaviour, snap to NDT), 0.3 = 30% new + 70%
  // old (rejects occasional NDT mis-lockings as small bumps rather than
  // instantaneous pose jumps visible downstream).
  double map_odom_smoothing_{0.3};

  // Optional post-NDT gate on the resulting map→odom correction. FAST_LIO
  // should carry short-term motion; NDT should only move map→odom slowly.
  // Disable with <= 0.0.
  double max_map_odom_update_translation_{-1.0};  // m
  double max_map_odom_update_rotation_{-1.0};     // rad

  // High-frequency re-broadcast of last_good_map_to_odom_ with node clock
  // stamps. Decouples TF freshness from the slower LiDAR scan rate so RViz
  // and Nav2 see map→base_link updated at ~50 Hz (limited only by FAST_LIO
  // odom→body), eliminating the visible RViz-lags-real-robot delay.
  rclcpp::TimerBase::SharedPtr tf_rebroadcast_timer_;
  rclcpp::CallbackGroup::SharedPtr tf_publish_group_;
  std::mutex tf_snapshot_mutex_;
  geometry_msgs::msg::TransformStamped tf_snapshot_;
  bool tf_snapshot_valid_{false};
  void updateTfSnapshot(bool valid);
  double tf_rebroadcast_rate_hz_{50.0};
  void rebroadcastMapOdomTF();
  bool use_imu_{false};
  bool enable_debug_{false};
  bool enable_map_odom_tf_{false};

  int ndt_num_threads_;
  int ndt_max_iterations_;
  bool adaptive_ndt_resolution_{true};
  double adaptive_ndt_max_resolution_{3.0};
  double adaptive_ndt_resolution_step_{0.1};
  double adaptive_ndt_min_usable_voxel_ratio_{0.01};

  double pose_publish_frequency_;
  bool enable_timer_publishing_{false};

  // imu
  LidarUndistortion lidar_undistortion_;

  rclcpp::TimerBase::SharedPtr pose_publish_timer_;
  void timerPublishPose();
};
