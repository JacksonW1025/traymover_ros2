#include "lidar_localization/lidar_localization_component.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace
{
using Cloud = pcl::PointCloud<pcl::PointXYZI>;

bool expect(bool condition, const char * message)
{
  if (!condition) {std::cerr << "FAILED: " << message << '\n';}
  return condition;
}

// Keep optimization deterministic while exercising the real source pipeline,
// PCL nearest-neighbor scores, bootstrap gate, and accepted pose publication.
class FixedRegistration : public pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>
{
public:
  Cloud aligned_input;
  Eigen::Matrix4f result{Eigen::Matrix4f::Identity()};
  int calls{0};

protected:
  void computeTransformation(PointCloudSource & output, const Matrix4 &) override
  {
    ++calls;
    aligned_input = *input_;
    final_transformation_ = result;
    converged_ = true;
    pcl::transformPointCloud(*input_, output, result);
  }
};

pcl::PointXYZI point(float x, float z)
{
  pcl::PointXYZI p;
  p.x = x;
  p.y = 0.0f;
  p.z = z;
  p.intensity = 1.0f;
  return p;
}

size_t count(const Cloud & cloud, float x, float z)
{
  return std::count_if(cloud.begin(), cloud.end(), [=](const pcl::PointXYZI & p) {
    return std::abs(p.x - x) < 1e-5f && std::abs(p.z - z) < 1e-5f;
  });
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  bool ok = true;
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"registration_method", "NDT_OMP"}, {"use_pcd_map", false},
      {"set_initial_pose", false}, {"use_imu", false}, {"use_odom", false},
      {"enable_timer_publishing", false}, {"ndt_align_interval_s", 0.0},
      {"voxel_leaf_size", 0.01}, {"scan_min_range", 1.0}, {"scan_max_range", 20.0},
      {"far_point_boost_min_range", 10.0}, {"far_point_boost_factor", 2},
      {"height_boost_enabled", true}, {"height_boost_min_z", 2.0},
      {"score_threshold", 0.1}});
    auto node = std::make_shared<PCLLocalization>(options);
    const rclcpp_lifecycle::State state;
    ok &= expect(node->on_configure(state) == PCLLocalization::CallbackReturn::SUCCESS,
      "valid height parameters configure successfully");
    node->on_activate(state);
    node->map_recieved_ = true;
    auto registration = boost::shared_ptr<FixedRegistration>(new FixedRegistration);
    node->registration_ = registration;

    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.transform.rotation.w = 1.0;
    node->tfbuffer_.setTransform(transform, "test", true);

    int stamp = 10;
    auto run = [&](const Cloud & source, const Cloud & target,
        const std::string & frame = "base_link") {
        auto initial = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
        initial->header.frame_id = "map";
        initial->header.stamp.sec = stamp;
        initial->pose.pose.orientation.w = 1.0;
        node->initialPoseReceived(initial);
        registration->setInputTarget(Cloud::Ptr(new Cloud(target)));
        auto scan = std::make_shared<sensor_msgs::msg::PointCloud2>();
        pcl::toROSMsg(source, *scan);
        scan->header.frame_id = frame;
        scan->header.stamp.sec = ++stamp;
        node->cloudReceived(scan);
        stamp += 10;
      };

    Cloud source;
    source.push_back(point(2, 0));
    source.push_back(point(2, 2));  // Inclusive height boundary.
    source.push_back(point(10, 0));
    source.push_back(point(10, 2));
    source.push_back(point(2, 2));  // Voxel filter must remove this before weighting.
    source.push_back(point(0.5, 3));
    source.push_back(point(21, 3));
    run(source, source);
    const auto & weighted = registration->aligned_input;
    ok &= expect(weighted.size() == 8 && count(weighted, 2, 0) == 1 &&
      count(weighted, 2, 2) == 2 && count(weighted, 10, 0) == 2 &&
      count(weighted, 10, 2) == 3,
      "base/height/far/combined weights are 1/2/2/3 after voxel and range filters");
    ok &= expect(node->have_good_pose_ && registration->getInputSource()->size() == 8,
      "successful baseline scoring restores the weighted registration source");

    node->height_boost_enabled_ = false;
    run(source, source);
    ok &= expect(registration->aligned_input.size() == 6 &&
      count(registration->aligned_input, 2, 2) == 1 &&
      count(registration->aligned_input, 10, 2) == 2,
      "disabled mode preserves original far-point weighting");
    node->height_boost_enabled_ = true;
    node->height_boost_min_z_ = 100.0;
    run(source, source);
    ok &= expect(registration->aligned_input.size() == 6 && node->have_good_pose_,
      "no eligible high points leaves the original source and acceptance intact");
    node->height_boost_min_z_ = 2.0;
    node->far_point_boost_factor_ = 4;
    run(source, source);
    ok &= expect(count(registration->aligned_input, 10, 2) == 4,
      "height weighting never reduces or multiplies existing far weighting above the cap");
    node->far_point_boost_factor_ = 2;

    transform.header.frame_id = "base_link";
    transform.child_frame_id = "test_laser";
    transform.transform.translation.z = 1.0;
    node->tfbuffer_.setTransform(transform, "test", true);
    Cloud laser, base;
    laser.push_back(point(2, 1.1f));
    base.push_back(point(2, 2.1f));
    run(laser, base, "test_laser");
    ok &= expect(registration->aligned_input.size() == 2 &&
      count(registration->aligned_input, 2, 2.1f) == 2,
      "height threshold uses base-frame coordinates, not sensor coordinates");

    Cloud score_source, target;
    score_source.push_back(point(10, 0));
    score_source.push_back(point(10, 3));
    target.push_back(point(11, 0));
    target.push_back(point(10, 3));
    node->score_threshold_ = 0.45;
    run(score_source, target);
    ok &= expect(!node->have_good_pose_ &&
      std::abs(registration->getFitnessScore() - 0.4) < 1e-6,
      "baseline fitness 0.5 rejects even when weighted fitness 0.4 would pass");
    ok &= expect(registration->getInputSource()->size() == 5,
      "rejected baseline scoring also restores the weighted source");

    // Evaluate a nonidentity alignment against a transformed target so the
    // baseline score cannot accidentally use source coordinates or stale TF.
    registration->result(0, 3) = 0.05f;
    target.clear();
    target.push_back(point(10.05f, 0));
    target.push_back(point(11.05f, 3));
    node->score_threshold_ = 0.55;
    run(score_source, target);
    ok &= expect(node->have_good_pose_ &&
      std::abs(registration->getFitnessScore() - 0.6) < 1e-5,
      "baseline fitness 0.5 accepts despite weighted fitness 0.6, using final transform");

    Cloud outside;
    outside.push_back(point(21, 3));
    const int calls = registration->calls;
    run(outside, outside);
    ok &= expect(!node->have_good_pose_ && registration->calls == calls,
      "empty range-filtered source cannot reuse a previous successful alignment");
    node->on_deactivate(state);
    node->on_cleanup(state);
  }
  for (double invalid : {-1.0, std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::quiet_NaN()})
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"height_boost_min_z", invalid}});
    auto node = std::make_shared<PCLLocalization>(options);
    ok &= expect(node->on_configure(rclcpp_lifecycle::State()) ==
      PCLLocalization::CallbackReturn::FAILURE, "invalid height parameters fail configuration");
  }
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
