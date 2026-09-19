#ifndef TRAYMOVER_ROBOT_SAFETY__OPTION20_HEALTH_PANEL_HPP_
#define TRAYMOVER_ROBOT_SAFETY__OPTION20_HEALTH_PANEL_HPP_

#include <memory>
#include <mutex>

#include <QLabel>
#include <QPlainTextEdit>
#include <QStringList>
#include <QTimer>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/string.hpp"

namespace traymover_robot_safety
{

struct Option20Health
{
  bool received{false};
  bool valid{false};
  bool ready_for_goal{false};
  bool ready{false};
  double stamp{0.0};
  double received_steady{0.0};
  QString error;
  QStringList reasons;
  QString command;
  QString metrics;
};

enum class Option20HealthState {Waiting, Invalid, Stale, GoalReady, GoalBlocked};

Option20Health parseOption20Health(const QByteArray & json, double received_steady);
Option20HealthState option20HealthState(
  const Option20Health & health, double ros_now, double steady_now);

class Option20HealthPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit Option20HealthPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  void refresh();

  std::mutex health_mutex_;
  Option20Health health_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr health_sub_;
  QLabel * goal_label_;
  QLabel * command_ready_label_;
  QPlainTextEdit * reasons_label_;
  QLabel * command_label_;
  QLabel * metrics_label_;
  QTimer * timer_;
};

}  // namespace traymover_robot_safety

#endif  // TRAYMOVER_ROBOT_SAFETY__OPTION20_HEALTH_PANEL_HPP_
