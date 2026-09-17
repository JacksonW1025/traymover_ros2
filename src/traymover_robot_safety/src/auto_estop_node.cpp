#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "traymover_robot_safety/depth_estop_evaluator.hpp"

namespace traymover_robot_safety
{
namespace
{
constexpr char kAutoEstopRequestTopic[] = "/traymover_estop/auto_request";
constexpr char kAutoEstopReasonTopic[] = "/traymover_estop/auto_reason";

const char * reasonName(DepthEstopReason reason)
{
  switch (reason) {
    case DepthEstopReason::kInvalidFrame:
      return "INVALID_FRAME";
    case DepthEstopReason::kLowValidDepth:
      return "LOW_VALID_DEPTH";
    case DepthEstopReason::kNearObstacle:
      return "NEAR_OBSTACLE";
    case DepthEstopReason::kNone:
    default:
      return "NONE";
  }
}
}

class AutoEstopNode : public rclcpp::Node
{
public:
  AutoEstopNode()
  : Node("traymover_auto_estop")
  {
    const std::string depth_topic = declare_parameter<std::string>(
      "depth_topic", "/camera/camera/depth/image_rect_raw");
    depth_timeout_sec_ = declare_parameter<double>("depth_timeout_sec", 0.80);
    heartbeat_rate_hz_ = declare_parameter<double>("heartbeat_rate_hz", 10.0);

    DepthEstopConfig config;
    config.depth_scale_m = declare_parameter<double>("depth_scale_m", 0.001);
    config.stop_distance_m = declare_parameter<double>("stop_distance_m", 0.8);
    config.release_distance_m = declare_parameter<double>("release_distance_m", 1.0);
    config.roi_x_min = declare_parameter<double>("roi_x_min", 0.25);
    config.roi_x_max = declare_parameter<double>("roi_x_max", 0.75);
    config.roi_y_min = declare_parameter<double>("roi_y_min", 0.20);
    config.roi_y_max = declare_parameter<double>("roi_y_max", 0.85);
    config.min_valid_fraction = declare_parameter<double>("min_valid_fraction", 0.50);
    config.severe_invalid_fraction =
      declare_parameter<double>("severe_invalid_fraction", 0.20);
    const int grid_columns = declare_parameter<int>("grid_columns", 16);
    const int grid_rows = declare_parameter<int>("grid_rows", 12);
    config.near_tile_fraction = declare_parameter<double>("near_tile_fraction", 0.20);
    const int min_near_tile_count = declare_parameter<int>("min_near_tile_count", 2);
    config.min_near_pixel_fraction =
      declare_parameter<double>("min_near_pixel_fraction", 0.005);
    const int obstacle_frame_count = declare_parameter<int>("obstacle_frame_count", 2);
    const int low_valid_frame_count = declare_parameter<int>("low_valid_frame_count", 2);
    const int clear_frame_count = declare_parameter<int>("clear_frame_count", 8);

    if (depth_timeout_sec_ <= 0.0 || heartbeat_rate_hz_ <= 0.0 ||
      grid_columns <= 0 || grid_rows <= 0 || min_near_tile_count <= 0 ||
      obstacle_frame_count <= 0 || low_valid_frame_count <= 0 ||
      clear_frame_count <= 0)
    {
      throw std::invalid_argument("timeouts, rates, grid sizes, and frame counts must be positive");
    }
    config.grid_columns = static_cast<std::size_t>(grid_columns);
    config.grid_rows = static_cast<std::size_t>(grid_rows);
    config.min_near_tile_count = static_cast<std::size_t>(min_near_tile_count);
    config.obstacle_frame_count = static_cast<std::size_t>(obstacle_frame_count);
    config.low_valid_frame_count = static_cast<std::size_t>(low_valid_frame_count);
    config.clear_frame_count = static_cast<std::size_t>(clear_frame_count);
    evaluator_ = std::make_unique<DepthEstopEvaluator>(config);

    request_pub_ = create_publisher<std_msgs::msg::Bool>(
      kAutoEstopRequestTopic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    reason_pub_ = create_publisher<std_msgs::msg::String>(
      kAutoEstopReasonTopic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr message) {
        onDepthImage(message);
      });

    const auto heartbeat_period = std::chrono::duration<double>(1.0 / heartbeat_rate_hz_);
    heartbeat_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(heartbeat_period),
      [this]() {publishHeartbeat();});

    publishRequest(true, "STARTUP");
    RCLCPP_INFO(
      get_logger(),
      "Automatic EStop started: topic=%s stop=%.2f m release=%.2f m timeout=%.2f s "
      "near_area=%.2f%% tiles=%zu frames=%zu",
      depth_topic.c_str(), config.stop_distance_m, config.release_distance_m,
      depth_timeout_sec_, config.min_near_pixel_fraction * 100.0,
      config.min_near_tile_count, config.obstacle_frame_count);
  }

private:
  void onDepthImage(const sensor_msgs::msg::Image::SharedPtr & image)
  {
    last_depth_time_ = std::chrono::steady_clock::now();
    have_depth_frame_ = true;
    const DepthEstopResult result = evaluator_->process(*image);
    if (!result.frame_valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Invalid depth frame; automatic EStop remains active");
    } else if (result.low_valid_depth) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Depth ROI valid fraction %.1f%% is below the safety minimum "
        "(confirmation %zu)",
        result.valid_fraction * 100.0, result.consecutive_low_valid_frames);
    }
    publishRequest(result.estop_requested, reasonName(result.reason), &result);
  }

  void publishHeartbeat()
  {
    const auto now = std::chrono::steady_clock::now();
    const double depth_age_sec = have_depth_frame_ ?
      std::chrono::duration<double>(now - last_depth_time_).count() :
      std::numeric_limits<double>::infinity();
    const bool stale = depth_age_sec > depth_timeout_sec_;
    if (stale) {
      evaluator_->forceEstop();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Depth image timeout: age=%.3f s limit=%.3f s; automatic EStop is active",
        depth_age_sec, depth_timeout_sec_);
    }
    publishRequest(
      evaluator_->estopRequested(), stale ? "DEPTH_TIMEOUT" : "HEARTBEAT");
  }

  void publishRequest(
    bool requested, const char * reason,
    const DepthEstopResult * result = nullptr)
  {
    std_msgs::msg::Bool message;
    message.data = requested;
    request_pub_->publish(message);
    // Observability only: never change the evaluator or the Bool request. A
    // low-valid observation must not leave a previous NEAR_OBSTACLE reason
    // masking a sensor fault, even during its frame-confirmation period.
    if (result != nullptr && result->low_valid_depth) {
      last_reason_ = "LOW_VALID_DEPTH";
    } else if (std::string(reason) != "HEARTBEAT" && std::string(reason) != "NONE") {
      last_reason_ = reason;
    } else if (!requested && std::string(reason) != "HEARTBEAT") {
      last_reason_ = "NONE";
    }
    // Keep the actual cause during heartbeat and clear-frame hysteresis.
    // NONE is emitted only once healthy frames have released the request.
    std_msgs::msg::String reason_message;
    reason_message.data = last_reason_;
    reason_pub_->publish(reason_message);
    if (!have_published_state_ || requested != last_published_state_) {
      if (requested && result != nullptr) {
        RCLCPP_INFO(
          get_logger(),
          "Automatic EStop state: ESTOP cause=%s valid=%.1f%% near=%.2f%% "
          "near_tiles=%zu obstacle_frames=%zu",
          reason, result->valid_fraction * 100.0,
          result->near_pixel_fraction * 100.0, result->near_tile_count,
          result->consecutive_obstacle_frames);
      } else {
        RCLCPP_INFO(
          get_logger(), "Automatic EStop state: %s cause=%s",
          requested ? "ESTOP" : "FREE", reason);
      }
      have_published_state_ = true;
      last_published_state_ = requested;
    }
  }

  std::unique_ptr<DepthEstopEvaluator> evaluator_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr request_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  std::chrono::steady_clock::time_point last_depth_time_{};
  double depth_timeout_sec_{0.80};
  double heartbeat_rate_hz_{10.0};
  bool have_depth_frame_{false};
  bool have_published_state_{false};
  bool last_published_state_{true};
  std::string last_reason_{"STARTUP"};
};

}  // namespace traymover_robot_safety

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<traymover_robot_safety::AutoEstopNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("traymover_auto_estop"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
