#include <gtest/gtest.h>

#include "alert_lamp/status_evaluator.hpp"

TEST(AlertLampCommandMapping, AutoNormalIsGreenSolid)
{
  const auto display = alert_lamp::StatusEvaluator().displayFor(alert_lamp::AlertState::AUTO_NORMAL,
    0.1F, 0.1F, 0.05F, 0.5F, {});
  EXPECT_EQ(display.color, alert_lamp::LampColor::GREEN);
  EXPECT_EQ(display.pattern, alert_lamp::LampPattern::SOLID);
  EXPECT_FLOAT_EQ(display.period, 0.0F);
  EXPECT_FLOAT_EQ(display.duty_ratio, 0.5F);
  EXPECT_FALSE(display.reason.empty());
}

TEST(AlertLampCommandMapping, ManualNormalIsYellowSolid)
{
  const auto display = alert_lamp::StatusEvaluator().displayFor(
    alert_lamp::AlertState::MANUAL_NORMAL, 0.1F, 0.1F, 0.05F, 0.5F, {});
  EXPECT_EQ(display.color, alert_lamp::LampColor::YELLOW);
  EXPECT_EQ(display.pattern, alert_lamp::LampPattern::SOLID);
}

TEST(AlertLampCommandMapping, AutonomyNotReadyBlinksGreenAndYellow)
{
  const auto display = alert_lamp::StatusEvaluator().displayFor(
    alert_lamp::AlertState::AUTONOMY_NOT_READY, 0.1F, 0.1F, 0.05F, 0.5F, {});
  EXPECT_EQ(display.color, alert_lamp::LampColor::GREEN_YELLOW);
  EXPECT_EQ(display.pattern, alert_lamp::LampPattern::BLINK);
  EXPECT_FLOAT_EQ(display.period, 0.1F);
}

TEST(AlertLampCommandMapping, GroundCommunicationLossUsesModeSpecificDualLamp)
{
  alert_lamp::SystemStatus automatic;
  automatic.mode = alert_lamp::OperatingMode::AUTO;
  const auto auto_display = alert_lamp::StatusEvaluator().displayFor(
    alert_lamp::AlertState::GROUND_COMMUNICATION_LOST, 0.1F, 0.1F, 0.05F, 0.5F, automatic);
  EXPECT_EQ(auto_display.color, alert_lamp::LampColor::GREEN_RED);
  EXPECT_EQ(auto_display.pattern, alert_lamp::LampPattern::SOLID);

  alert_lamp::SystemStatus manual;
  manual.mode = alert_lamp::OperatingMode::MANUAL;
  const auto manual_display = alert_lamp::StatusEvaluator().displayFor(
    alert_lamp::AlertState::GROUND_COMMUNICATION_LOST, 0.1F, 0.1F, 0.05F, 0.5F, manual);
  EXPECT_EQ(manual_display.color, alert_lamp::LampColor::YELLOW_RED);
  EXPECT_EQ(manual_display.pattern, alert_lamp::LampPattern::SOLID);
}
