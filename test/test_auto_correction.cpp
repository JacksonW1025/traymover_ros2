#include "lidar_localization/lidar_localization_component.hpp"

#include <cmath>
#include <functional>
#include <iostream>
#include <limits>

namespace
{
using Cloud = pcl::PointCloud<pcl::PointXYZI>;
using lidar_localization::AutoCorrectionPolicy;
using lidar_localization::CorrectionEvidence;
using lidar_localization::boundedCorrection;
using lidar_localization::plausibleOdom;
using lidar_localization::rotationDistance;

bool expect(bool condition, const char * message)
{
  if (!condition) {std::cerr << "FAILED: " << message << '\n';}
  return condition;
}

Eigen::Affine3d pose(double x = 0.0, double y = 0.0, double z = 0.0,
  double yaw = 0.0, double roll = 0.0)
{
  Eigen::Affine3d result = Eigen::Affine3d::Identity();
  result.translation() = Eigen::Vector3d(x, y, z);
  result.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  return result;
}

bool near(const Eigen::Matrix4f & actual, const Eigen::Affine3d & expected,
  double tolerance = 1e-5)
{
  return (actual.cast<double>() - expected.matrix()).norm() < tolerance;
}

Cloud sourceCloud()
{
  Cloud cloud;
  for (const Eigen::Vector3f & xyz : {Eigen::Vector3f(3.0f, 0.0f, 0.0f),
    Eigen::Vector3f(6.0f, 2.0f, 3.0f), Eigen::Vector3f(10.0f, -3.0f, 0.5f)})
  {
    pcl::PointXYZI point;
    point.x = xyz.x(); point.y = xyz.y(); point.z = xyz.z(); point.intensity = 1.0f;
    cloud.push_back(point);
  }
  return cloud;
}

// Only the optimizer is controlled: source preparation, NN fitness, all gates,
// scan-time TF lookup and final applied transform use the real component.
class FixedRegistration : public pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>
{
public:
  Eigen::Matrix4f result{Eigen::Matrix4f::Identity()};
  Eigen::Matrix4f guess{Eigen::Matrix4f::Identity()};
  bool succeeds{true};
  int calls{0};
  size_t source_count{0};
  std::function<void()> during_alignment;

protected:
  void computeTransformation(PointCloudSource & output, const Matrix4 & initial) override
  {
    ++calls;
    guess = initial;
    source_count = input_->size();
    final_transformation_ = result;
    converged_ = succeeds;
    pcl::transformPointCloud(*input_, output, result);
    if (during_alignment) {during_alignment();}
  }
};

struct Harness
{
  std::shared_ptr<PCLLocalization> node;
  boost::shared_ptr<FixedRegistration> registration;
  Cloud source{sourceCloud()};
  int stamp{10};
  bool configured{false};

  Harness()
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"registration_method", "NDT_OMP"}, {"use_pcd_map", false},
      {"set_initial_pose", false}, {"use_imu", false}, {"use_odom", false},
      {"enable_timer_publishing", false}, {"ndt_align_interval_s", 0.0},
      {"voxel_leaf_size", 0.01}, {"scan_min_range", 1.0}, {"scan_max_range", 30.0},
      {"far_point_boost_factor", 1}, {"score_threshold", 60.0},
      {"auto_correction_enabled", true},
      {"max_pose_jump_translation", 0.35}, {"max_pose_jump_rotation", 0.45},
      {"max_bootstrap_pose_jump_translation", 2.5},
      {"max_bootstrap_pose_jump_rotation", 0.6},
      {"max_map_odom_update_translation", 0.5},
      {"max_map_odom_update_rotation", 0.45}, {"map_odom_smoothing", 1.0}});
    node = std::make_shared<PCLLocalization>(options);
    configured = node->on_configure(rclcpp_lifecycle::State()) ==
      PCLLocalization::CallbackReturn::SUCCESS;
    if (!configured) {return;}
    node->on_activate(rclcpp_lifecycle::State());
    node->map_recieved_ = true;
    registration.reset(new FixedRegistration);
    node->registration_ = registration;
    odom(pose());
  }

  ~Harness()
  {
    if (configured) {
      node->on_deactivate(rclcpp_lifecycle::State());
      node->on_cleanup(rclcpp_lifecycle::State());
    }
  }

  void odom(const Eigen::Affine3d & transform, int at_stamp = 0)
  {
    auto message = tf2::eigenToTransform(transform);
    message.header.frame_id = "odom";
    message.child_frame_id = "base_link";
    message.header.stamp.sec = at_stamp;
    node->tfbuffer_.setTransform(message, "auto_correction_test", at_stamp == 0);
  }

  void initial(const Eigen::Affine3d & initial_pose = pose())
  {
    auto message = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    message->header.frame_id = "map";
    message->header.stamp.sec = stamp;
    message->pose.pose = tf2::toMsg(initial_pose);
    node->initialPoseReceived(message);
    registration->succeeds = true;
  }

  void scan(const Eigen::Affine3d & result, const Eigen::Affine3d & target_pose,
    const std::string & frame = "base_link", bool advance = true,
    const Cloud * target_cloud = nullptr)
  {
    if (advance) {++stamp;}
    registration->result = result.matrix().cast<float>();
    Cloud::Ptr target(new Cloud);
    if (target_cloud) {
      *target = *target_cloud;
    } else {
      pcl::transformPointCloud(source, *target, target_pose.matrix().cast<float>());
    }
    registration->setInputTarget(target);
    auto message = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(source, *message);
    message->header.frame_id = frame;
    message->header.stamp.sec = stamp;
    node->cloudReceived(message);
  }

  void bootstrap(const Eigen::Affine3d & initial_pose = pose())
  {
    initial(initial_pose);
    scan(initial_pose, initial_pose);
  }

  double appliedX() const {return node->last_good_map_to_odom_(0, 3);}
};

bool helperTests()
{
  bool ok = true;
  AutoCorrectionPolicy policy;
  ok &= expect(policy.valid(), "default auto correction policy is valid");
  AutoCorrectionPolicy invalid = policy;
  invalid.min_improvement = 0.0;
  ok &= expect(!invalid.valid(), "zero relative improvement cannot configure");
  invalid = policy;
  invalid.max_step_translation = std::numeric_limits<double>::quiet_NaN();
  ok &= expect(!invalid.valid(), "nonfinite step cannot configure");

  CorrectionEvidence evidence;
  ok &= expect(!evidence.confirm(pose(0.8), pose(), 1, policy) &&
    !evidence.confirm(pose(0.9), pose(), 2, policy) &&
    evidence.confirm(pose(0.85), pose(), 3, policy),
    "three candidates within a fixed anchor confirm");
  evidence.reset();
  for (int i = 0; i < 8; ++i) {
    ok &= expect(!evidence.confirm(pose(0.8 + 0.15 * i), pose(), i + 1, policy),
      "walking candidates cannot ratchet the confirmation anchor");
  }
  evidence.reset();
  evidence.confirm(pose(0.8), pose(), 1, policy);
  ok &= expect(!evidence.confirm(pose(0.8), pose(), 1, policy) && evidence.count == 0,
    "duplicate timestamps clear pending confirmation");
  evidence.confirm(pose(0.8), pose(), 2, policy);
  evidence.confirm(pose(0.8), pose(), 20, policy);
  ok &= expect(evidence.count == 1, "long observation gap restarts confirmation");

  const Eigen::Affine3d latest = pose(100, 20);
  const Eigen::Affine3d desired_pose = pose(100.8, 20, 0, 0.5);
  const Eigen::Affine3d candidate = desired_pose * latest.inverse();
  const Eigen::Affine3d bounded = boundedCorrection(pose(), candidate, latest, 1.0, policy);
  ok &= expect(((bounded * latest).translation() - latest.translation()).norm() <= 0.150001 &&
    rotationDistance(latest, bounded * latest) <= 0.080001,
    "translation and full rotation bounds apply at latest robot pose far from odom origin");
  ok &= expect((candidate.translation()).norm() > 20,
    "pivot regression actually exercises a large map-to-odom translation");
  ok &= expect(plausibleOdom(pose(), pose(0.5), 1, policy), "normal odom is accepted");
  ok &= expect(!plausibleOdom(pose(), pose(5), 1, policy) &&
    !plausibleOdom(pose(), pose(0, 0, 1), 1, policy) &&
    !plausibleOdom(pose(), pose(0, 0, 0, 2), 1, policy),
    "horizontal, vertical and angular odom discontinuities are rejected");
  return ok;
}

bool componentTests()
{
  bool ok = true;
  Harness h;
  if (!expect(h.configured, "component configures")) {return false;}
  ok &= expect(h.node->auto_correction_enabled_, "enable parameter reaches configured component");
  h.initial();
  h.scan(pose(0.8), pose(0.8));
  ok &= expect(h.node->have_good_pose_ && !h.node->ndt_bootstrap_pending_ &&
    std::abs(h.appliedX() - 0.8) < 1e-5,
    "bootstrap accepts its separate larger envelope immediately without tracking confirmation");
  h.bootstrap();
  h.scan(pose(0.8), pose(0.8));
  h.scan(pose(0.8), pose(0.8));
  ok &= expect(std::abs(h.appliedX()) < 1e-6 && h.node->correction_evidence_.count == 2,
    "large correction is held for consecutive confirmation");
  h.scan(pose(0.8), pose(0.8));
  ok &= expect(std::abs(h.appliedX() - 0.15) < 1e-5 &&
    std::abs(h.node->auto_prediction_map_odom_.translation().x() - 0.8) < 1e-5,
    "confirmation stores raw correction but applies only the configured translation step");
  Eigen::Affine3d applied_pose;
  tf2::fromMsg(h.node->corrent_pose_with_cov_stamped_ptr_->pose.pose, applied_pose);
  ok &= expect((applied_pose.matrix() - h.node->last_good_map_to_odom_.cast<double>()).norm() < 1e-5 &&
    std::abs(applied_pose.translation().x() - 0.15) < 1e-5,
    "published pose follows bounded TF rather than raw confirmed optimizer pose");
  for (int i = 0; i < 6; ++i) {
    const double before = h.appliedX();
    h.scan(pose(0.8), pose(0.8));
    ok &= expect(near(h.registration->guess, pose(0.8)) &&
      h.appliedX() >= before - 1e-6 && h.appliedX() - before <= 0.150001,
      "fresh matches start from raw accepted pose and smoothly catch up");
  }
  ok &= expect(std::abs(h.appliedX() - 0.8) < 1e-5,
    "bounded map transform converges after confirmed correction");

  h.bootstrap();
  for (int i = 0; i < 6; ++i) {h.scan(pose(i % 2 ? -0.8 : 0.8), pose(i % 2 ? -0.8 : 0.8));}
  ok &= expect(std::abs(h.appliedX()) < 1e-6, "alternating aliases never receive confirmation");
  h.bootstrap();
  for (int i = 0; i < 6; ++i) {h.scan(pose(0.8 + i * 0.15), pose(0.8 + i * 0.15));}
  ok &= expect(std::abs(h.appliedX()) < 1e-6, "gradually walking component candidates never confirm");

  h.bootstrap();
  h.scan(pose(0.8), pose(0.8));
  h.scan(pose(0.8), pose());
  ok &= expect(h.node->correction_evidence_.count == 0 && std::abs(h.appliedX()) < 1e-6,
    "high score threshold cannot substitute for same-scan fitness improvement");
  h.scan(pose(0.8), pose(0.8));
  h.scan(pose(0.8), pose(0.401));
  ok &= expect(h.node->correction_evidence_.count == 0,
    "improvement below one percent does not confirm even under a permissive score limit");
  h.scan(pose(0.8), pose(0.8));
  h.registration->succeeds = false;
  h.scan(pose(0.8), pose(0.8));
  ok &= expect(h.node->correction_evidence_.count == 0,
    "nonconvergence clears pending evidence");
  h.registration->succeeds = true;
  h.scan(pose(0.8), pose(0.8));
  Eigen::Affine3d nonfinite = pose();
  nonfinite.translation().x() = std::numeric_limits<double>::quiet_NaN();
  h.scan(nonfinite, pose(0.8));
  ok &= expect(h.node->correction_evidence_.count == 0,
    "nonfinite optimizer output clears pending evidence before scoring");
  h.scan(pose(0.8), pose(0.8));
  h.scan(pose(3.0), pose(3.0));
  ok &= expect(h.node->correction_evidence_.count == 0 && std::abs(h.appliedX()) < 1e-6,
    "absolute correction envelope rejects otherwise perfect large match");

  h.node->lock_planar_ = true;
  h.bootstrap();
  h.scan(pose(0.8), pose(0.8));
  h.scan(pose(0.8, 0, 0, 0, 0.9), pose(0.8, 0, 0, 0, 0.9));
  ok &= expect(h.node->correction_evidence_.count == 0 && std::abs(h.appliedX()) < 1e-6,
    "raw full-3D bad rotation cannot be hidden by planar projection");
  h.node->lock_planar_ = false;

  h.bootstrap();
  h.scan(pose(0.8), pose(0.8));
  h.node->odom_frame_id_ = "missing_odom";
  h.scan(pose(0.8), pose(0.8));
  ok &= expect(h.node->correction_evidence_.count == 0,
    "missing scan-time odom transform clears confirmation evidence");
  h.node->odom_frame_id_ = "odom";

  h.bootstrap();
  h.scan(pose(0.8), pose(0.8));
  h.scan(pose(0.8), pose(0.8), "missing_sensor");
  ok &= expect(h.node->correction_evidence_.count == 0,
    "missing scan transform clears confirmation evidence");
  h.scan(pose(0.8), pose(0.8));
  h.initial();
  ok &= expect(h.node->correction_evidence_.count == 0 && !h.node->have_good_pose_ &&
    !h.node->auto_odom_fault_ && h.node->auto_last_scan_stamp_ < 0,
    "manual reinitialization clears all automatic correction history");

  h.bootstrap();
  h.scan(pose(0.8), pose(0.8));
  h.scan(pose(0.8), pose(0.8), "base_link", false);
  h.scan(pose(0.8), pose(0.8));
  ok &= expect(std::abs(h.appliedX()) < 1e-6,
    "repeated delivery of same scan cannot satisfy multi-frame confirmation");

  h.node->height_boost_enabled_ = true;
  h.node->height_boost_min_z_ = 2.0;
  h.bootstrap();
  for (int i = 0; i < 3; ++i) {h.scan(pose(0.8), pose(0.8));}
  ok &= expect(std::abs(h.appliedX() - 0.15) < 1e-5 &&
    h.registration->source_count == 4 && h.registration->getInputSource()->size() == 4,
    "height weighting coexists with correction and restores weighted source after scoring");
  // Low point favors the prediction; high point favors the candidate equally.
  // Height-weighted fitness would improve 50%, while baseline fitness ties.
  h.source.resize(2);
  h.bootstrap();
  Cloud mixed_target = h.source;
  mixed_target[1].x += 0.8f;
  for (int i = 0; i < 3; ++i) {
    h.scan(pose(0.8), pose(), "base_link", true, &mixed_target);
  }
  ok &= expect(std::abs(h.appliedX()) < 1e-6 && h.node->correction_evidence_.count == 0,
    "prediction and candidate fitness use identical baseline population despite height boost");
  h.source = sourceCloud();
  h.node->height_boost_enabled_ = false;

  h.node->auto_correction_enabled_ = false;
  h.bootstrap();
  for (int i = 0; i < 4; ++i) {h.scan(pose(0.8), pose(0.8));}
  ok &= expect(std::abs(h.appliedX()) < 1e-6,
    "feature-off preserves original hard pose-jump rejection");
  h.node->max_pose_jump_translation_ = 2.0;
  h.scan(pose(0.8), pose(0.8));
  ok &= expect(std::abs(h.appliedX()) < 1e-6,
    "feature-off preserves original map-to-odom hard gate independently");
  return ok;
}

bool odomTests()
{
  bool ok = true;
  Harness h;
  if (!h.configured) {return false;}
  h.odom(pose(100));
  h.bootstrap(pose(100));
  h.scan(pose(100, 0, 0, 0.2), pose(100, 0, 0, 0.2));
  const Eigen::Affine3d applied(h.node->last_good_map_to_odom_.cast<double>());
  ok &= expect(rotationDistance(pose(100), applied * pose(100)) > 0.01 &&
    rotationDistance(pose(100), applied * pose(100)) <= 0.080001 &&
    ((applied * pose(100)).translation() - pose(100).translation()).norm() < 1e-4,
    "rotation near a distant odom origin bypasses origin-dependent legacy gate");

  // Static TF gives a controlled odom jump without requiring callback timing.
  h.odom(pose(110));
  h.scan(pose(110), pose(110));
  ok &= expect(h.node->auto_odom_fault_ && !h.node->have_good_pose_,
    "implausible odom jump latches a fault and invalidates pose");
  h.odom(pose(100));
  h.scan(pose(100), pose(100));
  ok &= expect(h.node->auto_odom_fault_ && !h.node->have_good_pose_,
    "returning odom does not silently clear a latched fault");
  h.bootstrap(pose(100));
  ok &= expect(!h.node->auto_odom_fault_ && h.node->have_good_pose_,
    "explicit reinitialization recovers from odom fault");

  // Rebuild TF as dynamic, with newer odometry already available at align time.
  h.node->tfbuffer_.clear();
  h.initial(pose());
  h.odom(pose(), h.stamp + 1);
  h.odom(pose(0.5), h.stamp + 2);
  h.scan(pose(), pose());
  h.odom(pose(1.0), h.stamp + 2);
  h.node->corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x = 500;
  h.scan(pose(0.5), pose(0.5));
  ok &= expect(near(h.registration->guess, pose(0.5)),
    "prediction uses exact scan TF instead of latest odom or callback-mutated pose");
  return ok;
}

bool alignmentAndConfigurationTests()
{
  bool ok = true;
  {
    Harness h;
    if (!h.configured) {return false;}
    h.bootstrap();
    h.registration->during_alignment = [&]() {h.odom(pose(10));};
    h.scan(pose(), pose());
    ok &= expect(h.node->auto_odom_fault_ && !h.node->have_good_pose_,
      "odom jump arriving during alignment is checked again before applying correction");
  }
  {
    Harness h;
    if (!h.configured) {return false;}
    h.node->tfbuffer_.clear();
    h.odom(pose(), h.stamp + 1);
    h.bootstrap();
    h.odom(pose(), h.stamp + 1);
    h.registration->during_alignment = [&]() {h.odom(pose(), h.stamp + 7);};
    h.scan(pose(0.1), pose(0.1));
    ok &= expect(h.node->have_good_pose_ && !h.node->auto_odom_fault_ &&
      near(h.node->last_good_map_to_odom_, pose()) &&
      (h.node->auto_prediction_map_odom_.matrix() - pose().matrix()).norm() < 1e-6,
      "stale post-alignment odom time skips correction without latching a motion fault");
  }
  {
    Harness h;
    if (!h.configured) {return false;}
    h.node->tfbuffer_.clear();
    h.odom(pose(), h.stamp + 1);
    h.bootstrap();
    const Eigen::Affine3d candidate = pose(0.8, 0, 0, 0.59);
    for (int i = 0; i < 2; ++i) {
      h.odom(pose(), h.stamp + 1);
      h.scan(candidate, candidate);
    }
    ok &= expect(h.node->correction_evidence_.count == 2,
      "current-position envelope regression reaches final confirmation");
    h.odom(pose(), h.stamp + 1);
    h.registration->during_alignment = [&]() {h.odom(pose(6), h.stamp + 3);};
    h.scan(candidate, candidate);
    ok &= expect(h.node->have_good_pose_ && !h.node->auto_odom_fault_ &&
      h.node->correction_evidence_.count == 0 &&
      near(h.node->last_good_map_to_odom_, pose()) &&
      (h.node->auto_prediction_map_odom_.matrix() - pose().matrix()).norm() < 1e-6,
      "robot motion expanding correction beyond current-position envelope discards confirmed result");
  }
  for (const rclcpp::Parameter & invalid : {
    rclcpp::Parameter("auto_correction_confirmations", 1),
    rclcpp::Parameter("auto_correction_max_translation", 0.0),
    rclcpp::Parameter("auto_correction_consistency_translation", 3.0),
    rclcpp::Parameter("auto_correction_min_improvement", 1.0),
    rclcpp::Parameter("map_odom_smoothing", 0.0),
    rclcpp::Parameter("enable_timer_publishing", true)})
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("auto_correction_enabled", true), invalid});
    auto node = std::make_shared<PCLLocalization>(options);
    ok &= expect(node->on_configure(rclcpp_lifecycle::State()) ==
      PCLLocalization::CallbackReturn::FAILURE,
      "invalid recovery policy or incompatible publication mode fails configuration");
  }
  return ok;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  bool ok = helperTests();
  ok &= componentTests();
  ok &= odomTests();
  ok &= alignmentAndConfigurationTests();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
