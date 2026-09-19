#include "lidar_localization/lidar_localization_component.hpp"

#include <cmath>
#include <iostream>
#include <limits>

namespace
{

bool expect(bool condition, const char * message)
{
  if (!condition) {
    std::cerr << "FAILED: " << message << '\n';
  }
  return condition;
}

// Exercise the production callback and its gates without depending on an
// optimizer's convergence on synthetic data. Fitness still uses PCL's real
// nearest-neighbor implementation against the supplied map.
class FixedRegistration : public pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>
{
public:
  Eigen::Matrix4f result{Eigen::Matrix4f::Identity()};
  int calls{0};

protected:
  void computeTransformation(PointCloudSource & output, const Matrix4 &) override
  {
    ++calls;
    final_transformation_ = result;
    converged_ = true;
    pcl::transformPointCloud(*input_, output, result);
  }
};

geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose(double x, int stamp)
{
  auto message = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  message->header.frame_id = "map";
  message->header.stamp.sec = stamp;
  message->pose.pose.position.x = x;
  message->pose.pose.orientation.w = 1.0;
  return message;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  bool ok = true;
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"registration_method", "NDT"}, {"use_pcd_map", false},
      {"set_initial_pose", false}, {"use_imu", false}, {"use_odom", true},
      {"enable_timer_publishing", false}, {"lock_planar", true},
      {"voxel_leaf_size", 0.01}, {"scan_min_range", 0.0},
      {"score_threshold", 0.1}, {"max_bootstrap_pose_jump_translation", 2.5},
      {"max_bootstrap_pose_jump_rotation", 0.6}, {"map_odom_smoothing", 0.3},
      {"max_map_odom_update_translation", 0.1},
      {"max_pose_jump_translation", 0.1}});
    auto node = std::make_shared<PCLLocalization>(options);
    const rclcpp_lifecycle::State state;
    node->on_configure(state);
    node->on_activate(state);
    node->map_recieved_ = true;

    auto registration = boost::shared_ptr<FixedRegistration>(new FixedRegistration);
    node->registration_ = registration;
    pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; i < 8; ++i) {
      pcl::PointXYZI point;
      point.x = 2.0f + 0.2f * i;
      point.y = 0.1f * (i % 3);
      point.z = 0.1f * (i % 2);
      point.intensity = 1.0f;
      source->push_back(point);
    }
    auto scan = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*source, *scan);
    scan->header.frame_id = "base_link";
    scan->header.stamp.sec = 10;
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.rotation.w = 1.0;
    node->tfbuffer_.setTransform(odom_tf, "test", true);

    auto set_result = [&](double x, double roll = 0.0, double z = 0.0) {
      registration->result = Eigen::Matrix4f::Identity();
      registration->result(0, 3) = x;
      registration->result(2, 3) = z;
      registration->result.block<3, 3>(0, 0) =
        Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()).toRotationMatrix();
      pcl::PointCloud<pcl::PointXYZI>::Ptr target(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::transformPointCloud(*source, *target, registration->result);
      registration->setInputTarget(target);
    };

    node->initialPoseReceived(pose(0.0, 1));
    set_result(1.0);
    node->cloudReceived(scan);
    ok &= expect(node->have_good_pose_ && !node->ndt_bootstrap_pending_,
      "bootstrap accepts correction within 2.5 m even beyond tracking gate");
    ok &= expect(std::abs(node->last_good_map_to_odom_(0, 3) - 1.0) < 1e-5,
      "bootstrap commits unsmoothed map->odom");

    const int calls = registration->calls;
    auto reset = pose(20.0, 20);
    node->initialPoseReceived(reset);
    ok &= expect(!node->have_good_pose_ && node->ndt_bootstrap_pending_,
      "manual reinit invalidates the accepted pose");
    ok &= expect(!node->last_odom_pose_valid_ && !node->planar_locked_,
      "manual reinit clears odom delta and plane lock");
    ok &= expect(registration->calls == calls && !node->last_scan_ptr_,
      "manual reinit does not reprocess a stale successful scan");
    node->cloudReceived(scan);
    ok &= expect(registration->calls == calls, "queued pre-reset scan is ignored");

    reset->pose.pose.position.x = 999.0;
    ok &= expect(node->corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x == 20.0,
      "initial pose is copied rather than aliased to caller");
    auto odom = std::make_shared<nav_msgs::msg::Odometry>();
    odom->header.stamp.sec = 19;
    odom->pose.pose.orientation.w = 1.0;
    odom->pose.pose.position.x = 1.0;
    node->odomReceived(odom);
    odom->header.stamp.nanosec = 100000000;
    odom->pose.pose.position.x = 2.0;
    node->odomReceived(odom);
    ok &= expect(!node->last_odom_pose_valid_ &&
      node->corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x == 20.0,
      "queued pre-reset odometry cannot move the new pose or set its baseline");
    odom->header.stamp.sec = 20;
    odom->header.stamp.nanosec = 0;
    odom->pose.pose.position.x = 100.0;
    node->odomReceived(odom);
    ok &= expect(node->corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x == 20.0,
      "first post-reset odom establishes baseline without old motion");
    odom->header.stamp.nanosec = 100000000;
    odom->pose.pose.position.x = 100.25;
    node->odomReceived(odom);
    ok &= expect(std::abs(node->corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x -
      20.25) < 1e-5, "following odom propagates new pose normally");
    odom->header.stamp.nanosec = 50000000;
    odom->pose.pose.position.x = 0.0;
    node->odomReceived(odom);
    ok &= expect(std::abs(node->last_odom_received_time_ - 20.1) < 1e-6 &&
      node->last_odom_pose_.position.x == 100.25,
      "out-of-order odom preserves both timestamp and pose baseline");

    scan->header.stamp.sec = 21;
    set_result(21.0);
    node->cloudReceived(scan);
    ok &= expect(node->have_good_pose_ && !node->ndt_bootstrap_pending_,
      "reinit succeeds far from old map->odom despite strict correction gate");
    ok &= expect(std::abs(node->last_good_map_to_odom_(0, 3) - 21.0) < 1e-5,
      "reinit result does not blend with old map->odom");

    auto invalid = pose(50.0, 25);
    invalid->header.frame_id = "odom";
    node->initialPoseReceived(invalid);
    invalid->header.frame_id = "map";
    invalid->pose.pose.orientation.w = 0.0;
    node->initialPoseReceived(invalid);
    invalid->pose.pose.orientation.w = 1.0;
    invalid->pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
    node->initialPoseReceived(invalid);
    node->initialPoseReceived(nullptr);
    invalid = pose(50.0, -1);
    node->initialPoseReceived(invalid);
    invalid = pose(50.0, 25);
    invalid->header.stamp.nanosec = 1000000000U;
    node->initialPoseReceived(invalid);
    ok &= expect(node->have_good_pose_ && !node->ndt_bootstrap_pending_ &&
      node->corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x == 21.0,
      "invalid frame, zero quaternion, NaN, and null initial poses preserve accepted state");

    node->initialPoseReceived(pose(30.0, 30));
    scan->header.stamp.sec = 31;
    set_result(33.0);
    node->cloudReceived(scan);
    ok &= expect(!node->have_good_pose_ && node->ndt_bootstrap_pending_ &&
      !node->planar_locked_, "bootstrap rejects >2.5 m jump without capturing plane");
    scan->header.stamp.sec = 40;
    set_result(30.0, 0.7);
    node->cloudReceived(scan);
    ok &= expect(!node->have_good_pose_ && !node->planar_locked_,
      "bootstrap rejects >0.6 rad roll, not only yaw");
    scan->header.stamp.sec = 50;
    set_result(30.5);
    node->cloudReceived(scan);
    ok &= expect(node->have_good_pose_ && node->planar_locked_,
      "later bounded bootstrap retry succeeds");
    scan->header.stamp.sec = 56;
    set_result(30.51, 0.0, 5.0);
    node->cloudReceived(scan);
    ok &= expect(std::abs(node->corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x -
      30.51) < 1e-5 &&
      std::abs(node->corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z) < 1e-5,
      "tracking projects raw Z drift onto established plane before jump check");
    const float previous_map_x = node->last_good_map_to_odom_(0, 3);
    // Once tracking, the same 0.5 m correction must face the strict 0.1 m gate.
    const int calls_before_tracking_rejection = registration->calls;
    scan->header.stamp.sec = 62;
    set_result(31.0);
    node->cloudReceived(scan);
    ok &= expect(registration->calls == calls_before_tracking_rejection + 1,
      "steady-state rejection test actually invokes alignment after interval");
    ok &= expect(std::abs(node->last_good_map_to_odom_(0, 3) - previous_map_x) < 1e-5,
      "steady-state jump protection resumes after bootstrap acceptance");
    auto normalized = pose(40.0, 70);
    normalized->pose.pose.orientation.w = 2.0;
    node->initialPoseReceived(normalized);
    ok &= expect(node->corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation.w == 1.0,
      "finite nonunit initial quaternion is normalized");
    scan->header.stamp.sec = 71;
    registration->result(0, 3) = std::numeric_limits<float>::quiet_NaN();
    node->cloudReceived(scan);
    ok &= expect(!node->have_good_pose_ && !node->planar_locked_,
      "nonfinite alignment is rejected before fitness nearest-neighbor search");
    node->on_deactivate(state);
    node->on_cleanup(state);
  }
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
