#include <limits>

#include <gtest/gtest.h>

#include <QJsonDocument>
#include <QJsonObject>

#include "traymover_robot_safety/option20_health_panel.hpp"

namespace safety = traymover_robot_safety;
using State = safety::Option20HealthState;

namespace
{
const QByteArray kReady = R"({"ready_for_goal":true,"ready":false,
  "reasons":["command:stale"],"stamp":100.0,"command":[0.0,0.0],
  "map_score":0.8,"finite_beams":91})";
}

TEST(Option20Health, IdleCommandDoesNotPreventGoalReadiness)
{
  const auto health = safety::parseOption20Health(kReady, 20.0);
  ASSERT_TRUE(health.valid);
  EXPECT_TRUE(health.ready_for_goal);
  EXPECT_FALSE(health.ready);
  EXPECT_EQ(health.reasons, QStringList{"command:stale"});
  EXPECT_EQ(safety::option20HealthState(health, 100.2, 20.2), State::GoalReady);
  EXPECT_TRUE(health.command.contains("v=0.000"));
  EXPECT_TRUE(health.metrics.contains("finite_beams=91"));
}

TEST(Option20Health, GoalBlockedIsIndependentOfCommandReady)
{
  auto json = QJsonDocument::fromJson(kReady).object();
  json["ready_for_goal"] = false;
  json["ready"] = true;
  const auto health = safety::parseOption20Health(QJsonDocument(json).toJson(), 20.0);
  EXPECT_EQ(safety::option20HealthState(health, 100.2, 20.2), State::GoalBlocked);
}

TEST(Option20Health, InvalidRequiredFieldsNeverDisplayGreen)
{
  const QByteArray cases[] = {
    "{", "[]", "null", "{}",
    R"({"ready_for_goal":"true","ready":false,"reasons":[],"stamp":100})",
    R"({"ready_for_goal":true,"ready":0,"reasons":[],"stamp":100})",
    R"({"ready_for_goal":true,"ready":false,"reasons":"none","stamp":100})",
    R"({"ready_for_goal":true,"ready":false,"reasons":[2],"stamp":100})",
    R"({"ready_for_goal":true,"ready":false,"reasons":[],"stamp":"100"})",
    R"({"ready_for_goal":true,"ready":false,"reasons":[],"stamp":0})",
    R"({"ready_for_goal":true,"ready":false,"reasons":[],"stamp":1e999})"
  };
  for (const auto & json : cases) {
    const auto health = safety::parseOption20Health(json, 20.0);
    EXPECT_FALSE(health.valid) << json.constData();
    EXPECT_EQ(safety::option20HealthState(health, 100.2, 20.2), State::Invalid);
  }
}

TEST(Option20Health, RequiresRecentReceiptEvenWhenRosClockPaused)
{
  const auto health = safety::parseOption20Health(kReady, 20.0);
  EXPECT_EQ(safety::option20HealthState(health, 100.0, 21.01), State::Stale);
  // A delayed Qt event cannot renew callback receipt time.
  EXPECT_EQ(safety::option20HealthState(health, 100.0, 25.0), State::Stale);
  EXPECT_EQ(safety::option20HealthState(health, 100.0, 19.9), State::Stale);
}

TEST(Option20Health, RejectsOldOrFutureRosStampAndInvalidClocks)
{
  const auto health = safety::parseOption20Health(kReady, 20.0);
  EXPECT_EQ(safety::option20HealthState(health, 101.01, 20.1), State::Stale);
  EXPECT_EQ(safety::option20HealthState(health, 99.8, 20.1), State::Stale);
  EXPECT_EQ(safety::option20HealthState(health, 99.95, 20.1), State::GoalReady);
  EXPECT_EQ(safety::option20HealthState(
      health, std::numeric_limits<double>::quiet_NaN(), 20.1), State::Stale);
}

TEST(Option20Health, WaitingAndRecoveryRequireActualNewData)
{
  EXPECT_EQ(safety::option20HealthState({}, 100.0, 20.0), State::Waiting);
  auto health = safety::parseOption20Health(kReady, 20.0);
  EXPECT_EQ(safety::option20HealthState(health, 100.5, 20.5), State::GoalReady);
  health = safety::parseOption20Health("invalid", 20.6);
  EXPECT_EQ(safety::option20HealthState(health, 100.6, 20.6), State::Invalid);
  health = safety::parseOption20Health(kReady, 20.7);
  EXPECT_EQ(safety::option20HealthState(health, 100.7, 20.7), State::GoalReady);
}

TEST(Option20Health, OptionalFieldsCannotCoerceWrongTypesToNumbers)
{
  auto json = QJsonDocument::fromJson(kReady).object();
  json["command"] = "[1,2]";
  json["map_score"] = "0.9";
  json["finite_beams"] = false;
  const auto health = safety::parseOption20Health(QJsonDocument(json).toJson(), 20.0);
  EXPECT_TRUE(health.valid);
  EXPECT_TRUE(health.command.contains("unavailable"));
  EXPECT_TRUE(health.metrics.isEmpty());
}
