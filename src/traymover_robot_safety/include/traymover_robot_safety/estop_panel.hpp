#ifndef TRAYMOVER_ROBOT_SAFETY__ESTOP_PANEL_HPP_
#define TRAYMOVER_ROBOT_SAFETY__ESTOP_PANEL_HPP_

#include <memory>

#include <QButtonGroup>
#include <QLabel>
#include <QPushButton>
#include <QTimer>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace traymover_robot_safety
{

class EstopPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit EstopPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void onEstopClicked(bool checked);
  void onFreeClicked(bool checked);
  void updateServiceAvailability();

private:
  void setUiState(bool estop);
  void updateStatusText();
  void requestEstop(bool estop);
  void onStateMessage(const std_msgs::msg::Bool::SharedPtr msg);
  void onManualStateMessage(const std_msgs::msg::Bool::SharedPtr msg);
  void onAutoStateMessage(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr set_estop_client_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr manual_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_state_sub_;

  QPushButton * estop_button_{nullptr};
  QPushButton * free_button_{nullptr};
  QLabel * status_label_{nullptr};
  QButtonGroup * button_group_{nullptr};
  QTimer * service_timer_{nullptr};

  bool estop_{false};
  bool manual_estop_{false};
  bool auto_estop_{false};
  bool service_ready_{false};
  bool updating_ui_{false};
};

}  // namespace traymover_robot_safety

#endif  // TRAYMOVER_ROBOT_SAFETY__ESTOP_PANEL_HPP_
