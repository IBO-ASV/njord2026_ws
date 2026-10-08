#include <gtest/gtest.h>

#include "frame_safety.hpp"

TEST(OmniFirmwareSafety, StopsCalibrationPulseWithoutAnotherCommand)
{
  EXPECT_FALSE(
    omni_md10c3::calibrationPulseExpired(true, true, 999U, 0U, 1000U));
  EXPECT_TRUE(
    omni_md10c3::calibrationPulseExpired(true, true, 1000U, 0U, 1000U));
  EXPECT_FALSE(
    omni_md10c3::calibrationPulseExpired(false, true, 1000U, 0U, 1000U));
}

TEST(OmniFirmwareSafety, RejectsSlowOrStalePartialFrame)
{
  // A byte arriving every 40 ms passes the inter-byte condition, but the
  // complete frame is discarded when its total age reaches 50 ms.
  EXPECT_TRUE(
    omni_md10c3::partialFrameExpired(true, 80U, 0U, 40U, 50U, 50U));
  EXPECT_TRUE(
    omni_md10c3::partialFrameExpired(true, 51U, 0U, 0U, 50U, 50U));
}

TEST(OmniFirmwareSafety, RecoversAfterTimeoutAtTheNextFrameBoundary)
{
  omni_md10c3::FrameRecoveryState state;

  // The tail of the timed-out frame is rejected until its delimiter.
  EXPECT_TRUE(state.onWatchdog(true, true));
  EXPECT_TRUE(state.commandTimeoutActive());
  EXPECT_FALSE(state.acceptsData());
  EXPECT_FALSE(state.onDelimiter());
  EXPECT_TRUE(state.acceptsData());

  // A fresh COBS frame may arrive over several loop passes after that
  // delimiter; it is not discarded merely because the old command timed out.
  EXPECT_FALSE(state.onWatchdog(true, true));
  EXPECT_TRUE(state.acceptsData());
  EXPECT_TRUE(state.onDelimiter());
  state.onValidCommand();
  EXPECT_FALSE(state.commandTimeoutActive());
  EXPECT_FALSE(state.onWatchdog(false, false));
}

TEST(OmniFirmwareSafety, UsesRolloverSafeDeadline)
{
  EXPECT_TRUE(omni_md10c3::elapsedAtLeast(4U, 0xFFFFFFF0U, 20U));
  EXPECT_FALSE(omni_md10c3::elapsedAtLeast(4U, 0xFFFFFFF0U, 21U));
}
