#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "traymover_robot_nav/srv/update_option22_path.hpp"

namespace traymover_robot_nav {
class Option22GlobalRoute : public BT::ControlNode {
public:
  using BT::ControlNode::ControlNode;

  static BT::PortsList providedPorts() {
    return {BT::BidirectionalPort<std::vector<geometry_msgs::msg::PoseStamped>>("remaining_goals")};
  }

  BT::NodeStatus tick() override {
    if (childrenCount() != 2) {
      throw BT::LogicError("Option22GlobalRoute requires planning and tracking children");
    }
    geometry_msgs::msg::PoseStamped goal;
    std::vector<geometry_msgs::msg::PoseStamped> goals;
    config().blackboard->get("goal", goal);
    config().blackboard->get("goals", goals);
    if (status() == BT::NodeStatus::IDLE || goal != goal_ || goals != goals_) {
      halt();
      goal_ = goal;
      goals_ = std::move(goals);
      if (config().output_ports.count("remaining_goals")) {
        setOutput("remaining_goals", goals_);
      }
    }
    setStatus(BT::NodeStatus::RUNNING);
    if (!planned_) {
      const auto result = children_nodes_[0]->executeTick();
      if (result != BT::NodeStatus::SUCCESS) {
        return result;
      }
      planned_ = true;
    }

    // Nav2 reports the size of {goals} as waypoint feedback. Progress on
    // the existing route can shrink that list without invalidating the plan.
    std::vector<geometry_msgs::msg::PoseStamped> remaining;
    if (getInput("remaining_goals", remaining) && remaining != goals_) {
      goals_ = std::move(remaining);
      config().blackboard->set("goals", goals_);
    }
    const auto result = children_nodes_[1]->executeTick();
    if (result != BT::NodeStatus::RUNNING) {
      halt();
    }
    return result;
  }

  void halt() override {
    planned_ = false;
    BT::ControlNode::halt();
  }

private:
  geometry_msgs::msg::PoseStamped goal_;
  std::vector<geometry_msgs::msg::PoseStamped> goals_;
  bool planned_{false};
};

class Option22UpdatePath : public nav2_behavior_tree::BtServiceNode<srv::UpdateOption22Path> {
public:
  Option22UpdatePath(const std::string & name, const BT::NodeConfiguration & config)
      : BtServiceNode(name, config) {}

  static BT::PortsList providedPorts() {
    return providedBasicPorts({
      BT::InputPort<nav_msgs::msg::Path>("global_path"),
      BT::OutputPort<nav_msgs::msg::Path>("path"),
    });
  }

  BT::NodeStatus tick() override {
    const auto now = std::chrono::steady_clock::now();
    if (status() == BT::NodeStatus::IDLE) {
      restart_tracking_ = true;
      next_check_ = now;
    }
    if (now < next_check_) {
      return BT::NodeStatus::RUNNING;
    }
    const auto result = BtServiceNode::tick();
    if (result == BT::NodeStatus::SUCCESS) {
      next_check_ = now + std::chrono::milliseconds(250);
    }
    return result;
  }

  void on_tick() override {
    should_send_request_ = getInput("global_path", request_->global_path) &&
      !request_->global_path.poses.empty();
    request_->restart_tracking = restart_tracking_;
    restart_tracking_ = false;
  }

  BT::NodeStatus on_completion(std::shared_ptr<srv::UpdateOption22Path::Response> result) override {
    setOutput("path", result->tracking_path);
    return result->tracking_path.poses.empty() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }

private:
  std::chrono::steady_clock::time_point next_check_{};
  bool restart_tracking_{true};
};
}  // namespace traymover_robot_nav

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<traymover_robot_nav::Option22GlobalRoute>("Option22GlobalRoute");
  factory.registerNodeType<traymover_robot_nav::Option22UpdatePath>("Option22UpdatePath");
}
