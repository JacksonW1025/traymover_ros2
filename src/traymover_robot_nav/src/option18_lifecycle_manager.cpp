// Option 18 startup recovery; retain Nav2's manager services and bond watchdog.
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "nav2_lifecycle_manager/lifecycle_manager.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

class Option18LifecycleManager : public nav2_lifecycle_manager::LifecycleManager
{
public:
  Option18LifecycleManager()
  {
    const bool start_requested = autostart_;
    // The base initialization timer still creates clients and its service thread.
    autostart_ = false;
    timeout_ = std::chrono::seconds(declare_parameter("startup_service_timeout_sec", 5));
    attempts_ = declare_parameter("startup_attempts", 3);
    if (timeout_.count() <= 0 || attempts_ <= 0 || bond_timeout_.count() <= 0) {
      throw std::invalid_argument("Startup timeouts, attempts and bond_timeout must be positive");
    }
    if (start_requested) {
      startup_timer_ = create_wall_timer(100ms, [this]() {
          startup_timer_->cancel();
          bringup();
        }, callback_group_);
    }
  }

  ~Option18LifecycleManager()
  {
    // Join the base callback thread before destroying derived callback state.
    service_thread_.reset();
  }

private:
  bool wait_for_localization()
  {
    // Nav2's global costmap blocks activation until this transform exists.
    // Waiting for the operator's initial pose is not a failed lifecycle service.
    // This runs after map_server activation so RViz can display the map first.
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    // bringup runs on the manager's service thread; the main executor remains
    // free to process TF. Avoid another executor/thread during node shutdown.
    // Keep the buffer/listener alive until BOTH executors have stopped. A
    // stack-local buffer could be destroyed here while the main executor is
    // still finishing a TF callback during shutdown.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(
      *tf_buffer_, shared_from_this(), false);
    auto next_log = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero)) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_log) {
        RCLCPP_INFO(get_logger(),
          "Waiting for initial localization (map -> base_link). "
          "Set 2D Pose Estimate in RViz; navigation activation will continue automatically.");
        next_log = now + 5s;
      }
      std::this_thread::sleep_for(100ms);
    }
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_INFO(get_logger(), "Initial localization is available; continuing navigation activation");
    return true;
  }

  bool reach_state(const std::string & name, uint8_t transition, uint8_t target)
  {
    for (int attempt = 0; attempt < attempts_ && rclcpp::ok(); ++attempt) {
      try {
        const auto state = node_map_.at(name)->get_state(timeout_);
        if (state == target ||
          (target == State::PRIMARY_STATE_INACTIVE && state == State::PRIMARY_STATE_ACTIVE))
        {
          return true;
        }
        const auto source = transition == Transition::TRANSITION_CONFIGURE ?
          State::PRIMARY_STATE_UNCONFIGURED : State::PRIMARY_STATE_INACTIVE;
        // A lost response does not mean the transition failed. Never send another
        // transition while GetState says the node is still changing state.
        if (state == source) {
          node_map_.at(name)->change_state(transition, timeout_);
        }
        const auto deadline = std::chrono::steady_clock::now() + timeout_;
        do {
          const auto observed = node_map_.at(name)->get_state(timeout_);
          if (observed == target) {
            return true;
          }
          if (observed < State::TRANSITION_STATE_CONFIGURING) {
            break;
          }
          std::this_thread::sleep_for(200ms);
        } while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline);
      } catch (const std::exception & error) {
        RCLCPP_WARN(get_logger(), "%s startup attempt %d/%d: %s; checking state again",
          name.c_str(), attempt + 1, attempts_, error.what());
      }
      std::this_thread::sleep_for(200ms);
    }
    return false;
  }

  bool connect_bond(const std::string & name)
  {
    // bondcpp 3.0.2 waitUntilFormed reads its state machine without its mutex.
    // Its formed callback avoids that race while retaining the native watchdog.
    auto formed = std::make_shared<std::atomic<bool>>(false);
    auto connection = std::make_shared<bond::Bond>("bond", name, shared_from_this(),
      std::function<void()>(), [formed]() {formed->store(true);});
    connection->setHeartbeatTimeout(std::chrono::duration<double>(bond_timeout_).count());
    connection->setHeartbeatPeriod(0.10);
    bond_map_[name] = connection;
    connection->start();
    const auto deadline = std::chrono::steady_clock::now() + bond_timeout_;
    while (rclcpp::ok() && !formed->load() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(10ms);
    }
    if (!formed->load() || connection->isBroken()) {
      RCLCPP_ERROR(get_logger(), "%s did not establish its lifecycle bond", name.c_str());
      return false;
    }
    return true;
  }

  void stop_partial_startup()
  {
    // Fail closed after bounded retries, including a partially activated stack.
    for (auto it = node_names_.rbegin(); it != node_names_.rend() && rclcpp::ok(); ++it) {
      try {
        if (node_map_.at(*it)->get_state(timeout_) == State::PRIMARY_STATE_ACTIVE) {
          node_map_.at(*it)->change_state(Transition::TRANSITION_DEACTIVATE, timeout_);
        }
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Cannot deactivate %s after startup failure: %s",
          it->c_str(), error.what());
      }
    }
    RCLCPP_ERROR(get_logger(), "Option 18 navigation startup failed; navigation is not ready");
  }

  void bringup()
  {
    RCLCPP_INFO(get_logger(), "Starting managed nodes with bounded state-checked transitions");
    for (const auto & name : node_names_) {
      if (!reach_state(name, Transition::TRANSITION_CONFIGURE, State::PRIMARY_STATE_INACTIVE)) {
        stop_partial_startup();
        return;
      }
    }
    for (const auto & name : node_names_) {
      if (name == "planner_server" && !wait_for_localization()) {
        return;
      }
      if (!reach_state(name, Transition::TRANSITION_ACTIVATE, State::PRIMARY_STATE_ACTIVE) ||
        !connect_bond(name))
      {
        stop_partial_startup();
        return;
      }
    }
    system_active_ = true;
    createBondTimer();
    RCLCPP_INFO(get_logger(), "Managed nodes are active; lifecycle bond monitoring enabled");
  }

  rclcpp::TimerBase::SharedPtr startup_timer_;
  std::chrono::seconds timeout_{5};
  int attempts_{3};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Option18LifecycleManager>());
  rclcpp::shutdown();
  return 0;
}
