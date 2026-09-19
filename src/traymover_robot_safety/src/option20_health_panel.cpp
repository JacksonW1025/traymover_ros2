#include "traymover_robot_safety/option20_health_panel.hpp"

#include <chrono>
#include <cmath>

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMetaObject>
#include <QVBoxLayout>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace
{
double steadySeconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool finiteNumber(const QJsonValue & value)
{
  return value.isDouble() && std::isfinite(value.toDouble());
}

QLabel * plainLabel(const QString & name)
{
  auto * label = new QLabel;
  label->setObjectName(name);
  label->setTextFormat(Qt::PlainText);
  label->setWordWrap(true);
  label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  return label;
}
}  // namespace

namespace traymover_robot_safety
{

Option20Health parseOption20Health(const QByteArray & json, double received_steady)
{
  Option20Health health;
  health.received = true;
  health.received_steady = received_steady;
  QJsonParseError error;
  const auto document = QJsonDocument::fromJson(json, &error);
  if (error.error != QJsonParseError::NoError || !document.isObject()) {
    health.error = "Message is not a valid JSON object";
    return health;
  }
  const auto object = document.object();
  if (!object.value("ready_for_goal").isBool() || !object.value("ready").isBool() ||
    !object.value("reasons").isArray() || !finiteNumber(object.value("stamp")) ||
    object.value("stamp").toDouble() <= 0.0)
  {
    health.error = "Required: bool ready_for_goal / ready, string array reasons, numeric stamp";
    return health;
  }
  for (const auto & reason : object.value("reasons").toArray()) {
    if (!reason.isString()) {
      health.error = "Every reason must be a string";
      return health;
    }
    health.reasons.append(reason.toString());
  }
  health.ready_for_goal = object.value("ready_for_goal").toBool();
  health.ready = object.value("ready").toBool();
  health.stamp = object.value("stamp").toDouble();
  const auto command = object.value("command").toArray();
  health.command = "Command: unavailable";
  if (command.size() == 2 && finiteNumber(command[0]) && finiteNumber(command[1])) {
    health.command = QString("Command: v=%1 m/s, w=%2 rad/s")
      .arg(command[0].toDouble(), 0, 'f', 3).arg(command[1].toDouble(), 0, 'f', 3);
  }
  QStringList metrics;
  if (finiteNumber(object.value("map_score"))) {
    metrics.append(QString("map_score=%1").arg(object.value("map_score").toDouble(), 0, 'f', 3));
  }
  if (finiteNumber(object.value("finite_beams"))) {
    metrics.append(QString("finite_beams=%1").arg(object.value("finite_beams").toDouble()));
  }
  health.metrics = metrics.join("  ");
  health.valid = true;
  return health;
}

Option20HealthState option20HealthState(
  const Option20Health & health, double ros_now, double steady_now)
{
  if (!health.received) {
    return Option20HealthState::Waiting;
  }
  if (!health.valid) {
    return Option20HealthState::Invalid;
  }
  const double ros_age = ros_now - health.stamp;
  const double receive_age = steady_now - health.received_steady;
  if (!std::isfinite(ros_age) || !std::isfinite(receive_age) || ros_age < -0.1 ||
    ros_age > 1.0 || receive_age < 0.0 || receive_age > 1.0)
  {
    return Option20HealthState::Stale;
  }
  return health.ready_for_goal ? Option20HealthState::GoalReady : Option20HealthState::GoalBlocked;
}

Option20HealthPanel::Option20HealthPanel(QWidget * parent)
: rviz_common::Panel(parent),
  goal_label_(plainLabel("option20_ready_for_goal")),
  command_ready_label_(plainLabel("option20_ready")),
  reasons_label_(new QPlainTextEdit),
  command_label_(plainLabel("option20_command")),
  metrics_label_(plainLabel("option20_metrics")),
  timer_(new QTimer(this))
{
  auto * layout = new QVBoxLayout;
  layout->setContentsMargins(4, 4, 4, 4);
  layout->setSpacing(3);
  reasons_label_->setObjectName("option20_reasons");
  reasons_label_->setReadOnly(true);
  reasons_label_->setMaximumHeight(60);
  auto * topic_label = plainLabel("option20_topic");
  topic_label->setText("/option20/health (read only)");
  goal_label_->setWordWrap(false);
  goal_label_->setToolTip(
    "ready_for_goal: TRUE means a goal can be sent, not that the robot is moving.\n"
    "UNKNOWN: check the health topic and ROS clocks.");
  command_ready_label_->setToolTip(
    "ready: whether velocity commands may pass.\n"
    "While idle, FALSE with command:stale can be normal even when ready_for_goal is TRUE.");
  layout->addWidget(topic_label);
  layout->addWidget(goal_label_);
  layout->addWidget(command_ready_label_);
  layout->addWidget(reasons_label_);
  layout->addWidget(command_label_);
  layout->addWidget(metrics_label_);
  layout->addStretch();
  setLayout(layout);
  connect(timer_, &QTimer::timeout, this, &Option20HealthPanel::refresh);
  timer_->start(200);
  refresh();
}

void Option20HealthPanel::onInitialize()
{
  const auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    return;
  }
  node_ = abstraction->get_raw_node();
  health_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/option20/health", rclcpp::QoS(1).reliable().durability_volatile(),
    [this](const std_msgs::msg::String::SharedPtr message) {
      // Capture receipt time in the ROS callback, never when a queued UI update runs.
      const double received = steadySeconds();
      auto health = parseOption20Health(QByteArray::fromStdString(message->data), received);
      if (option20HealthState(health, node_->now().seconds(), received) ==
        Option20HealthState::Stale)
      {
        // A future message must not become green later without a new publication.
        health.valid = false;
        health.error = "Timestamp is stale or differs from the RViz ROS clock";
      }
      {
        const std::lock_guard<std::mutex> lock(health_mutex_);
        health_ = std::move(health);
      }
      QMetaObject::invokeMethod(this, [this]() {refresh();}, Qt::QueuedConnection);
    });
}

void Option20HealthPanel::refresh()
{
  Option20Health health;
  {
    const std::lock_guard<std::mutex> lock(health_mutex_);
    health = health_;
  }
  const auto state = option20HealthState(
    health, node_ ? node_->now().seconds() : 0.0, steadySeconds());
  const bool current = state == Option20HealthState::GoalReady ||
    state == Option20HealthState::GoalBlocked;
  QString value = "UNKNOWN";
  QString color = "#5c626b";
  QString explanation = "Waiting for health message";
  if (state == Option20HealthState::GoalReady) {
    value = "TRUE";
    color = "#176b36";
  } else if (state == Option20HealthState::GoalBlocked) {
    value = "FALSE";
    color = "#a94712";
  } else if (state == Option20HealthState::Invalid) {
    color = "#8c3030";
    explanation = "Invalid message: " + health.error;
  } else if (state == Option20HealthState::Stale) {
    color = "#8c3030";
    explanation = "Health stale or clock mismatch (1 second timeout)";
  }
  goal_label_->setText("Ready for goal: " + value);
  goal_label_->setStyleSheet(QString(
      "QLabel { background-color: %1; color: white; font-weight: bold; "
      "font-size: 16px; padding: 6px; }").arg(color));
  command_ready_label_->setText(current ?
    QString("Command gate (ready): %1").arg(health.ready ? "TRUE" : "FALSE") :
    "Command gate (ready): UNKNOWN");
  const QString reasons = current ? "Reasons:\n" +
    (health.reasons.isEmpty() ? QString("none") : health.reasons.join("\n")) : explanation;
  if (reasons_label_->toPlainText() != reasons) {
    reasons_label_->setPlainText(reasons);
  }
  command_label_->setText(current ? health.command : "Command: UNKNOWN");
  metrics_label_->setText(current ? health.metrics : QString());
}

}  // namespace traymover_robot_safety

PLUGINLIB_EXPORT_CLASS(traymover_robot_safety::Option20HealthPanel, rviz_common::Panel)
