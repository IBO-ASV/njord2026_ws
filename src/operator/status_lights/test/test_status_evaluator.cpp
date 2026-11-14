#include <gtest/gtest.h>

#include "status_lights/status_evaluator.hpp"

namespace
{
status_lights::SystemStatus normal(status_lights::OperatingMode mode)
{
  status_lights::SystemStatus s;
  s.mode = mode; s.state_unknown = false; s.driver_alive = true; s.high_level_alive = true;
  s.autonomy_alive = true; s.localization_alive = true; s.manual_control_alive = true;
  s.ground_station_connected = true; s.autonomy_ready = true; s.localization_stable = true;
  s.rtk_fix = true;
  return s;
}
}  // namespace

TEST(StatusEvaluator, ManualNormal)
{
  EXPECT_EQ(status_lights::StatusEvaluator().evaluate(normal(status_lights::OperatingMode::MANUAL)),
    status_lights::AlertState::MANUAL_NORMAL);
}
TEST(StatusEvaluator, AutoNormal)
{
  EXPECT_EQ(status_lights::StatusEvaluator().evaluate(normal(status_lights::OperatingMode::AUTO)),
    status_lights::AlertState::AUTO_NORMAL);
}
TEST(StatusEvaluator, AutoNormalDoesNotRequireLocalization)
{
  auto s = normal(status_lights::OperatingMode::AUTO);
  s.localization_alive = false;
  s.localization_stable = false;
  EXPECT_EQ(status_lights::StatusEvaluator().evaluate(s), status_lights::AlertState::AUTO_NORMAL);
}
TEST(StatusEvaluator, RtkMissingIsNotReady)
{
  auto s = normal(status_lights::OperatingMode::AUTO); s.rtk_fix = false;
  EXPECT_EQ(status_lights::StatusEvaluator().evaluate(s),
    status_lights::AlertState::AUTONOMY_NOT_READY);
}
TEST(StatusEvaluator, HighLevelMissingInAutoIsCritical)
{
  auto s = normal(status_lights::OperatingMode::AUTO); s.high_level_alive = false;
  EXPECT_EQ(status_lights::StatusEvaluator().evaluate(s),
    status_lights::AlertState::CRITICAL_FAULT);
}
TEST(StatusEvaluator, GroundLostSafe)
{
  auto s = normal(status_lights::OperatingMode::AUTO); s.ground_station_connected = false;
  EXPECT_EQ(status_lights::StatusEvaluator().evaluate(s),
    status_lights::AlertState::GROUND_COMMUNICATION_LOST);
}
TEST(StatusEvaluator, GroundLostLocalizationBadIsCritical)
{
  auto s = normal(status_lights::OperatingMode::AUTO); s.ground_station_connected = false;
    s.localization_stable = false;
  EXPECT_EQ(status_lights::StatusEvaluator().evaluate(s),
    status_lights::AlertState::CRITICAL_FAULT);
}
TEST(StatusEvaluator, CriticalConditionsWin)
{
  auto s = normal(status_lights::OperatingMode::AUTO); s.emergency_stop = true;
    s.ground_station_connected = false;
  EXPECT_EQ(status_lights::StatusEvaluator().evaluate(s),
    status_lights::AlertState::CRITICAL_FAULT);
}
TEST(StatusEvaluator, UnknownStateIsCritical)
{
  auto s = normal(status_lights::OperatingMode::AUTO); s.state_unknown = true;
  EXPECT_EQ(status_lights::StatusEvaluator().evaluate(s),
    status_lights::AlertState::CRITICAL_FAULT);
}
