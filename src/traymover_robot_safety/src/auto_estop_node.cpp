#include <chrono>
#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "traymover_robot_safety/depth_estop_evaluator.hpp"

namespace traymover_robot_safety
{
namespace
{
constexpr char kAutoEstopRequestTopic[] = "/traymover_estop/auto_request";
}

class AutoEstopNode : public rclcpp::Node
{
public:
  AutoEstopNode()
  : Node("traymover_auto_estop")
  {
    const std::string depth_topic = declare_parameter<std::string>(
      "depth_topic", "/camera/camera/depth/image_rect_raw");
    depth_timeout_sec_ = declare_parameter<double>("depth_timeout_sec", 0.50);
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
    min_valid_fraction_ = config.min_valid_fraction;
    const int grid_columns = declare_parameter<int>("grid_columns", 16);
    const int grid_rows = declare_parameter<int>("grid_rows", 12);
    config.near_tile_fraction = declare_parameter<double>("near_tile_fraction", 0.10);
    const int clear_frame_count = declare_parameter<int>("clear_frame_count", 8);

    if (depth_timeout_sec_ <= 0.0 || heartbeat_rate_hz_ <= 0.0 ||
      grid_columns <= 0 || grid_rows <= 0 || clear_frame_count <= 0)
    {
      throw std::invalid_argument("timeouts, rates, grid sizes, and frame counts must be positive");
    }
    config.grid_columns = static_cast<std::size_t>(grid_columns);
    config.grid_rows = static_cast<std::size_t>(grid_rows);
    config.clear_frame_count = static_cast<std::size_t>(clear_frame_count);
    evaluator_ = std::make_unique<DepthEstopEvaluator>(config);

    request_pub_ = create_publisher<std_msgs::msg::Bool>(
      kAutoEstopRequestTopic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
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

    publishRequest(true);
    RCLCPP_INFO(
      get_logger(),
      "Automatic EStop started: topic=%s stop=%.2f m release=%.2f m timeout=%.2f s",
      depth_topic.c_str(), config.stop_distance_m, config.release_distance_m,
      depth_timeout_sec_);
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
    } else if (result.valid_fraction < min_valid_fraction_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Depth ROI valid fraction %.1f%% is below the safety minimum",
        result.valid_fraction * 100.0);
    }
    publishRequest(result.estop_requested);
  }

  void publishHeartbeat()
  {
    const auto now = std::chrono::steady_clock::now();
    const bool stale = !have_depth_frame_ ||
      std::chrono::duration<double>(now - last_depth_time_).count() > depth_timeout_sec_;
    if (stale) {
      evaluator_->forceEstop();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Depth image timeout; automatic EStop is active");
    }
    publishRequest(evaluator_->estopRequested());
  }

  void publishRequest(bool requested)
  {
    std_msgs::msg::Bool message;
    message.data = requested;
    request_pub_->publish(message);
    if (!have_published_state_ || requested != last_published_state_) {
      RCLCPP_INFO(
        get_logger(), "Automatic EStop state: %s", requested ? "ESTOP" : "FREE");
      have_published_state_ = true;
      last_published_state_ = requested;
    }
  }

  std::unique_ptr<DepthEstopEvaluator> evaluator_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr request_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  std::chrono::steady_clock::time_point last_depth_time_{};
  double depth_timeout_sec_{0.50};
  double heartbeat_rate_hz_{10.0};
  double min_valid_fraction_{0.50};
  bool have_depth_frame_{false};
  bool have_published_state_{false};
  bool last_published_state_{true};
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
