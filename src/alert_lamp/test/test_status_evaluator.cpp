#include <gtest/gtest.h>

#include "alert_lamp/status_evaluator.hpp"

namespace
{
alert_lamp::SystemStatus normal(alert_lamp::OperatingMode mode)
{
  alert_lamp::SystemStatus s;
  s.mode = mode; s.state_unknown = false; s.driver_alive = true; s.high_level_alive = true;
  s.autonomy_alive = true; s.localization_alive = true; s.manual_control_alive = true;
  s.ground_station_connected = true; s.autonomy_ready = true; s.localization_stable = true;
  s.rtk_fix = true;
  return s;
}
}  // namespace

TEST(StatusEvaluator, ManualNormal)
{
  EXPECT_EQ(alert_lamp::StatusEvaluator().evaluate(normal(alert_lamp::OperatingMode::MANUAL)),
    alert_lamp::AlertState::MANUAL_NORMAL);
}
TEST(StatusEvaluator, AutoNormal)
{
  EXPECT_EQ(alert_lamp::StatusEvaluator().evaluate(normal(alert_lamp::OperatingMode::AUTO)),
    alert_lamp::AlertState::AUTO_NORMAL);
}
TEST(StatusEvaluator, AutoNormalDoesNotRequireLocalization)
{
  auto s = normal(alert_lamp::OperatingMode::AUTO);
  s.localization_alive = false;
  s.localization_stable = false;
  EXPECT_EQ(alert_lamp::StatusEvaluator().evaluate(s), alert_lamp::AlertState::AUTO_NORMAL);
}
TEST(StatusEvaluator, RtkMissingIsNotReady)
{
  auto s = normal(alert_lamp::OperatingMode::AUTO); s.rtk_fix = false;
  EXPECT_EQ(alert_lamp::StatusEvaluator().evaluate(s), alert_lamp::AlertState::AUTONOMY_NOT_READY);
}
TEST(StatusEvaluator, HighLevelMissingInAutoIsCritical)
{
  auto s = normal(alert_lamp::OperatingMode::AUTO); s.high_level_alive = false;
  EXPECT_EQ(alert_lamp::StatusEvaluator().evaluate(s), alert_lamp::AlertState::CRITICAL_FAULT);
}
TEST(StatusEvaluator, GroundLostSafe)
{
  auto s = normal(alert_lamp::OperatingMode::AUTO); s.ground_station_connected = false;
  EXPECT_EQ(alert_lamp::StatusEvaluator().evaluate(s),
    alert_lamp::AlertState::GROUND_COMMUNICATION_LOST);
}
TEST(StatusEvaluator, GroundLostLocalizationBadIsCritical)
{
  auto s = normal(alert_lamp::OperatingMode::AUTO); s.ground_station_connected = false;
    s.localization_stable = false;
  EXPECT_EQ(alert_lamp::StatusEvaluator().evaluate(s), alert_lamp::AlertState::CRITICAL_FAULT);
}
TEST(StatusEvaluator, CriticalConditionsWin)
{
  auto s = normal(alert_lamp::OperatingMode::AUTO); s.emergency_stop = true;
    s.ground_station_connected = false;
  EXPECT_EQ(alert_lamp::StatusEvaluator().evaluate(s), alert_lamp::AlertState::CRITICAL_FAULT);
}
TEST(StatusEvaluator, UnknownStateIsCritical)
{
  auto s = normal(alert_lamp::OperatingMode::AUTO); s.state_unknown = true;
  EXPECT_EQ(alert_lamp::StatusEvaluator().evaluate(s), alert_lamp::AlertState::CRITICAL_FAULT);
}
