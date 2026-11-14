#include <gtest/gtest.h>

#include "status_lights/status_evaluator.hpp"

TEST(AlertLampCommandMapping, AutoNormalIsGreenSolid)
{
  const auto display =
    status_lights::StatusEvaluator().displayFor(status_lights::AlertState::AUTO_NORMAL,
    0.1F, 0.1F, 0.05F, 0.5F, {});
  EXPECT_EQ(display.color, status_lights::LampColor::GREEN);
  EXPECT_EQ(display.pattern, status_lights::LampPattern::SOLID);
  EXPECT_FLOAT_EQ(display.period, 0.0F);
  EXPECT_FLOAT_EQ(display.duty_ratio, 0.5F);
  EXPECT_FALSE(display.reason.empty());
}

TEST(AlertLampCommandMapping, ManualNormalIsYellowSolid)
{
  const auto display = status_lights::StatusEvaluator().displayFor(
    status_lights::AlertState::MANUAL_NORMAL, 0.1F, 0.1F, 0.05F, 0.5F, {});
  EXPECT_EQ(display.color, status_lights::LampColor::YELLOW);
  EXPECT_EQ(display.pattern, status_lights::LampPattern::SOLID);
}

TEST(AlertLampCommandMapping, AutonomyNotReadyBlinksGreenAndYellow)
{
  const auto display = status_lights::StatusEvaluator().displayFor(
    status_lights::AlertState::AUTONOMY_NOT_READY, 0.1F, 0.1F, 0.05F, 0.5F, {});
  EXPECT_EQ(display.color, status_lights::LampColor::GREEN_YELLOW);
  EXPECT_EQ(display.pattern, status_lights::LampPattern::BLINK);
  EXPECT_FLOAT_EQ(display.period, 0.1F);
}

TEST(AlertLampCommandMapping, GroundCommunicationLossUsesModeSpecificDualLamp)
{
  status_lights::SystemStatus automatic;
  automatic.mode = status_lights::OperatingMode::AUTO;
  const auto auto_display = status_lights::StatusEvaluator().displayFor(
    status_lights::AlertState::GROUND_COMMUNICATION_LOST, 0.1F, 0.1F, 0.05F, 0.5F, automatic);
  EXPECT_EQ(auto_display.color, status_lights::LampColor::GREEN_RED);
  EXPECT_EQ(auto_display.pattern, status_lights::LampPattern::SOLID);

  status_lights::SystemStatus manual;
  manual.mode = status_lights::OperatingMode::MANUAL;
  const auto manual_display = status_lights::StatusEvaluator().displayFor(
    status_lights::AlertState::GROUND_COMMUNICATION_LOST, 0.1F, 0.1F, 0.05F, 0.5F, manual);
  EXPECT_EQ(manual_display.color, status_lights::LampColor::YELLOW_RED);
  EXPECT_EQ(manual_display.pattern, status_lights::LampPattern::SOLID);
}
