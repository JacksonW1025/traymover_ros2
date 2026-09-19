#include "lidar_localization/lidar_localization_component.hpp"
#include <tf2_msgs/msg/tf_message.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

namespace
{
using Cloud = pcl::PointCloud<pcl::PointXYZI>;
using SteadyClock = std::chrono::steady_clock;

bool expect(bool condition, const char * message)
{
  if (!condition) {std::cerr << "FAILED: " << message << '\n';}
  return condition;
}

class FixedRegistration : public pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>
{
public:
  int calls{0};
  std::function<void()> during_alignment;

protected:
  void computeTransformation(PointCloudSource & output, const Matrix4 &) override
  {
    ++calls;
    if (during_alignment) {during_alignment();}
    final_transformation_ = Matrix4::Identity();
    converged_ = true;
    output = *input_;
  }
};

struct Harness
{
  std::shared_ptr<PCLLocalization> node;
  boost::shared_ptr<FixedRegistration> registration;
  Cloud source;
  bool configured{false};
  bool active{false};

  explicit Harness(double interval)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"registration_method", "NDT_OMP"}, {"use_pcd_map", false},
      {"set_initial_pose", false}, {"use_imu", false}, {"use_odom", false},
      {"enable_timer_publishing", false}, {"ndt_align_interval_s", interval},
      {"voxel_leaf_size", 0.01}, {"scan_min_range", 1.0}, {"scan_max_range", 30.0},
      {"far_point_boost_factor", 1}, {"score_threshold", 60.0},
      {"auto_correction_enabled", true}, {"scan_odom_tf_timeout", 0.5},
      {"tf_rebroadcast_rate_hz", 50.0}, {"map_odom_smoothing", 1.0}});
    node = std::make_shared<PCLLocalization>(options);
    configured = node->on_configure(rclcpp_lifecycle::State()) ==
      PCLLocalization::CallbackReturn::SUCCESS;
    if (!configured) {return;}
    active = node->on_activate(rclcpp_lifecycle::State()) ==
      PCLLocalization::CallbackReturn::SUCCESS;
    registration.reset(new FixedRegistration);
    node->registration_ = registration;
    node->map_recieved_ = true;
    for (int i = 0; i < 3; ++i) {
      pcl::PointXYZI point;
      point.x = 3.0f + 3.0f * i;
      point.y = static_cast<float>(i);
      point.z = 0.5f * i;
      point.intensity = 1.0f;
      source.push_back(point);
    }
    registration->setInputTarget(Cloud::Ptr(new Cloud(source)));
  }

  ~Harness()
  {
    if (active) {deactivate();}
    if (configured) {node->on_cleanup(rclcpp_lifecycle::State());}
  }

  void deactivate()
  {
    node->on_deactivate(rclcpp_lifecycle::State());
    active = false;
  }

  void odom(double seconds, bool is_static = false)
  {
    auto transform = tf2::eigenToTransform(Eigen::Affine3d::Identity());
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    transform.header.stamp = rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
    node->tfbuffer_.setTransform(transform, "tf_responsiveness_test", is_static);
  }

  void initial()
  {
    auto message = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    message->header.frame_id = "map";
    message->pose.pose.orientation.w = 1.0;
    node->initialPoseReceived(message);
  }

  void scan(double seconds)
  {
    auto message = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(source, *message);
    message->header.frame_id = "base_link";
    message->header.stamp = rclcpp::Time(static_cast<int64_t>(seconds * 1e9));
    node->cloudReceived(message);
  }

  bool snapshotValid()
  {
    std::lock_guard<std::mutex> lock(node->tf_snapshot_mutex_);
    return node->tf_snapshot_valid_;
  }
};

struct Observation
{
  SteadyClock::time_point received;
  int64_t stamp;
};

bool concurrentPublicationTest()
{
  Harness h(0.0);
  if (!expect(h.configured && h.active, "component configures and activates")) {return false;}
  h.odom(0.0, true);
  h.initial();
  h.scan(10.0);
  bool ok = expect(h.node->have_good_pose_ && h.snapshotValid(),
    "real bootstrap callback enables accepted TF snapshot");

  std::mutex observations_mutex;
  std::vector<Observation> observations;
  auto observer = std::make_shared<rclcpp::Node>("tf_responsiveness_observer");
  auto subscription = observer->create_subscription<tf2_msgs::msg::TFMessage>(
    "/tf", rclcpp::QoS(100).reliable(),
    [&](const tf2_msgs::msg::TFMessage::SharedPtr message) {
      std::lock_guard<std::mutex> lock(observations_mutex);
      for (const auto & transform : message->transforms) {
        if (transform.header.frame_id == "map" && transform.child_frame_id == "odom") {
          observations.push_back({SteadyClock::now(),
            rclcpp::Time(transform.header.stamp).nanoseconds()});
        }
      }
    });
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  rclcpp::executors::SingleThreadedExecutor observer_executor;
  executor.add_node(h.node->get_node_base_interface());
  observer_executor.add_node(observer);
  std::thread executor_thread([&]() {executor.spin();});
  std::thread observer_thread([&]() {observer_executor.spin();});

  const auto discovery_deadline = SteadyClock::now() + std::chrono::seconds(5);
  bool discovered = false;
  while (SteadyClock::now() < discovery_deadline) {
    {
      std::lock_guard<std::mutex> lock(observations_mutex);
      discovered = observations.size() >= 3;
    }
    if (discovered) {break;}
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ok &= expect(discovered, "independent observer receives map-to-odom before blocking test");

  SteadyClock::time_point blocked_start, blocked_end;
  int64_t blocked_stamp_start = 0;
  h.registration->during_alignment = [&]() {
    blocked_start = SteadyClock::now();
    blocked_stamp_start = h.node->now().nanoseconds();
    std::this_thread::sleep_for(std::chrono::milliseconds(800));
    blocked_end = SteadyClock::now();
  };
  std::promise<void> completed;
  auto completion = completed.get_future();
  rclcpp::TimerBase::SharedPtr trigger;
  std::atomic<bool> triggered{false};
  // Same mutually exclusive default group as the production cloud subscription.
  trigger = h.node->create_wall_timer(std::chrono::milliseconds(100), [&]() {
      if (triggered.exchange(true)) {return;}
      h.scan(11.0);
      completed.set_value();
    });
  const bool finished = completion.wait_for(std::chrono::seconds(5)) == std::future_status::ready;
  ok &= expect(finished, "blocking registration callback completes");
  executor.cancel();
  executor_thread.join();
  trigger->cancel();

  if (finished) {
    std::vector<Observation> during;
    {
      std::lock_guard<std::mutex> lock(observations_mutex);
      for (const auto & observation : observations) {
        if (observation.received >= blocked_start && observation.received < blocked_end &&
          observation.stamp > blocked_stamp_start)
        {
          during.push_back(observation);
        }
      }
    }
    ok &= expect(during.size() >= 5,
      "at least five fresh TF stamps arrive while actual registration blocks default group");
    for (size_t i = 1; i < during.size(); ++i) {
      ok &= expect(during[i].stamp > during[i - 1].stamp,
        "rebroadcast timestamps advance during blocked alignment");
    }
  }

  h.initial();
  ok &= expect(!h.snapshotValid(), "new initial pose invalidates accepted snapshot");
  // Exercise the timer path after invalidation; the snapshot must remain disabled.
  h.node->rebroadcastMapOdomTF();
  ok &= expect(!h.snapshotValid(), "rebroadcast cannot restore invalidated snapshot");
  h.registration->during_alignment = nullptr;
  h.scan(12.0);
  ok &= expect(h.snapshotValid(), "new bootstrap restores snapshot");
  h.deactivate();
  ok &= expect(!h.snapshotValid(), "deactivation invalidates accepted snapshot");
  h.node->rebroadcastMapOdomTF();
  ok &= expect(!h.snapshotValid(), "rebroadcast stays disabled after deactivation");
  const int calls_before_inactive_scan = h.registration->calls;
  h.scan(13.0);
  h.node->timerPublishPose();
  ok &= expect(!h.snapshotValid() && h.registration->calls == calls_before_inactive_scan,
    "inactive subscriptions cannot align a cloud or re-enable TF publication");
  observer_executor.cancel();
  observer_thread.join();
  return ok;
}

bool delayedScanTransformTest()
{
  Harness h(2.0);
  if (!expect(h.configured && h.active, "delayed-TF component configures")) {return false;}
  h.odom(10.0);
  h.initial();
  std::thread producer([&]() {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      h.odom(11.0);
    });
  h.scan(11.0);
  producer.join();
  bool ok = expect(h.registration->calls == 1 && h.node->have_good_pose_ &&
    !h.node->ndt_bootstrap_pending_,
    "exact scan-time odom arriving after 200ms is accepted within configured timeout");
  const double previous_alignment = h.node->last_align_stamp_s_;
  h.scan(13.0);  // No odom at this stamp: bounded wait must fail without consuming the interval.
  ok &= expect(h.registration->calls == 1 &&
    h.node->last_align_stamp_s_ == previous_alignment,
    "missing exact-time TF does not call optimizer or consume NDT alignment interval");
  h.odom(13.1);
  h.scan(13.1);
  ok &= expect(h.registration->calls == 2 && h.node->have_good_pose_ &&
    std::abs(h.node->last_align_stamp_s_ - 13.1) < 1e-6,
    "valid scan 100ms after failed attempt retries instead of waiting another two seconds");
  return ok;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  bool ok = concurrentPublicationTest();
  ok &= delayedScanTransformTest();
  rclcpp::shutdown();
  return ok ? 0 : 1;
}
