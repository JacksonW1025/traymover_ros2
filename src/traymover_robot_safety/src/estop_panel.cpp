#include "traymover_robot_safety/estop_panel.hpp"

#include <memory>

#include <QHBoxLayout>
#include <QMetaObject>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace
{
constexpr char kEstopService[] = "/traymover_estop/set";
constexpr char kEstopStateTopic[] = "/traymover_estop/state";
constexpr char kManualEstopStateTopic[] = "/traymover_estop/manual_state";
constexpr char kAutoEstopStateTopic[] = "/traymover_estop/auto_state";
}  // namespace

namespace traymover_robot_safety
{

EstopPanel::EstopPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout;
  auto * buttons = new QHBoxLayout;

  estop_button_ = new QPushButton("EStop");
  free_button_ = new QPushButton("Free");
  estop_button_->setCheckable(true);
  free_button_->setCheckable(true);
  estop_button_->setMinimumHeight(40);
  free_button_->setMinimumHeight(40);
  estop_button_->setStyleSheet(
    "QPushButton:checked { background-color: #b00020; color: white; font-weight: bold; }");
  free_button_->setStyleSheet(
    "QPushButton:checked { background-color: #0b6b2b; color: white; font-weight: bold; }");

  button_group_ = new QButtonGroup(this);
  button_group_->setExclusive(true);
  button_group_->addButton(estop_button_);
  button_group_->addButton(free_button_);

  buttons->addWidget(estop_button_);
  buttons->addWidget(free_button_);
  layout->addLayout(buttons);

  status_label_ = new QLabel("Waiting for /traymover_estop/state");
  status_label_->setWordWrap(true);
  layout->addWidget(status_label_);
  setLayout(layout);

  connect(estop_button_, SIGNAL(clicked(bool)), this, SLOT(onEstopClicked(bool)));
  connect(free_button_, SIGNAL(clicked(bool)), this, SLOT(onFreeClicked(bool)));

  setUiState(false);
  status_label_->setText("Waiting for /traymover_estop/state");
}

void EstopPanel::onInitialize()
{
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    status_label_->setText("RViz ROS node unavailable");
    return;
  }

  node_ = abstraction->get_raw_node();
  set_estop_client_ = node_->create_client<std_srvs::srv::SetBool>(kEstopService);
  const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    kEstopStateTopic,
    state_qos,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      this->onStateMessage(msg);
    });
  manual_state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    kManualEstopStateTopic,
    state_qos,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      this->onManualStateMessage(msg);
    });
  auto_state_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
    kAutoEstopStateTopic,
    state_qos,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      this->onAutoStateMessage(msg);
    });

  service_timer_ = new QTimer(this);
  connect(service_timer_, SIGNAL(timeout()), this, SLOT(updateServiceAvailability()));
  service_timer_->start(500);
  updateServiceAvailability();
}

void EstopPanel::onEstopClicked(bool checked)
{
  if (updating_ui_ || !checked) {
    return;
  }
  requestEstop(true);
}

void EstopPanel::onFreeClicked(bool checked)
{
  if (updating_ui_ || !checked) {
    return;
  }
  requestEstop(false);
}

void EstopPanel::updateServiceAvailability()
{
  if (!set_estop_client_) {
    return;
  }

  service_ready_ = set_estop_client_->service_is_ready();
  updateStatusText();
}

void EstopPanel::setUiState(bool estop)
{
  updating_ui_ = true;
  const QSignalBlocker estop_blocker(estop_button_);
  const QSignalBlocker free_blocker(free_button_);

  estop_ = estop;
  estop_button_->setChecked(estop);
  free_button_->setChecked(!estop);

  updating_ui_ = false;
  updateStatusText();
}

void EstopPanel::updateStatusText()
{
  QString state;
  if (!estop_) {
    state = "Free";
  } else if (manual_estop_ && auto_estop_) {
    state = "EStop active: Manual + Automatic";
  } else if (manual_estop_) {
    state = "EStop active: Manual";
  } else if (auto_estop_) {
    state = "EStop active: Automatic";
  } else {
    state = "EStop active";
  }
  status_label_->setText(
    service_ready_ ? state + " - service ready" : state + " - service unavailable");
}

void EstopPanel::requestEstop(bool estop)
{
  if (!set_estop_client_ || !set_estop_client_->service_is_ready()) {
    service_ready_ = false;
    setUiState(estop_);
    status_label_->setText("EStop service unavailable");
    return;
  }

  // The aggregate state topic is authoritative. Restore the current state
  // until the chassis driver confirms the request.
  setUiState(estop_);
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = estop;
  (void)set_estop_client_->async_send_request(request);
  status_label_->setText(estop ? "EStop requested" : "Free requested");
}

void EstopPanel::onStateMessage(const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool estop = msg->data;
  QMetaObject::invokeMethod(
    this,
    [this, estop]() {
      setUiState(estop);
      updateServiceAvailability();
    },
    Qt::QueuedConnection);
}

void EstopPanel::onManualStateMessage(const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool estop = msg->data;
  QMetaObject::invokeMethod(
    this,
    [this, estop]() {
      manual_estop_ = estop;
      updateStatusText();
    },
    Qt::QueuedConnection);
}

void EstopPanel::onAutoStateMessage(const std_msgs::msg::Bool::SharedPtr msg)
{
  const bool estop = msg->data;
  QMetaObject::invokeMethod(
    this,
    [this, estop]() {
      auto_estop_ = estop;
      updateStatusText();
    },
    Qt::QueuedConnection);
}

}  // namespace traymover_robot_safety

PLUGINLIB_EXPORT_CLASS(traymover_robot_safety::EstopPanel, rviz_common::Panel)
