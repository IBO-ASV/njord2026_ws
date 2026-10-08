#ifndef OMNI_MD10C3_FRAME_SAFETY_HPP_
#define OMNI_MD10C3_FRAME_SAFETY_HPP_

#include <stdint.h>

// This header is deliberately independent of Arduino APIs so the timing
// boundary used by the firmware can be exercised by a host-side unit test.
namespace omni_md10c3
{

inline bool elapsedAtLeast(uint32_t now, uint32_t then, uint32_t interval)
{
  // Unsigned subtraction preserves the intended elapsed duration across the
  // 32-bit millis() rollover.
  return static_cast<uint32_t>(now - then) >= interval;
}

inline bool partialFrameExpired(
  bool has_partial_frame,
  uint32_t now,
  uint32_t frame_started_at,
  uint32_t last_byte_at,
  uint32_t inter_byte_limit,
  uint32_t total_frame_limit)
{
  return has_partial_frame &&
         (elapsedAtLeast(now, last_byte_at, inter_byte_limit) ||
         elapsedAtLeast(now, frame_started_at, total_frame_limit));
}

// Tracks the delimiter recovery boundary separately from the watchdog.  When
// the watchdog fires while a frame is partial, only the suffix of *that*
// frame is discarded.  A frame that starts after its delimiter is allowed to
// arrive over later loop iterations and may clear the watchdog once valid.
class FrameRecoveryState
{
public:
  bool onWatchdog(bool command_timed_out, bool has_partial_frame)
  {
    const bool timeout_transition = command_timed_out && !command_timeout_active_;
    command_timeout_active_ = command_timed_out;
    if (timeout_transition && has_partial_frame) {
      discardUntilDelimiter();
      return true;
    }
    return false;
  }

  void onValidCommand()
  {
    command_timeout_active_ = false;
  }

  void discardUntilDelimiter()
  {
    discarding_until_delimiter_ = true;
  }

  // Returns true only when the delimiter belongs to a frame that should be
  // decoded.  A discarded suffix's delimiter just restores the parser.
  bool onDelimiter()
  {
    if (discarding_until_delimiter_) {
      discarding_until_delimiter_ = false;
      return false;
    }
    return true;
  }

  bool acceptsData() const
  {
    return !discarding_until_delimiter_;
  }

  bool commandTimeoutActive() const
  {
    return command_timeout_active_;
  }

private:
  bool command_timeout_active_ = false;
  bool discarding_until_delimiter_ = false;
};

inline bool calibrationPulseExpired(
  bool calibration_enabled,
  bool pulse_active,
  uint32_t now,
  uint32_t pulse_started_at,
  uint32_t pulse_limit)
{
  return calibration_enabled && pulse_active &&
         elapsedAtLeast(now, pulse_started_at, pulse_limit);
}

}  // namespace omni_md10c3

#endif  // OMNI_MD10C3_FRAME_SAFETY_HPP_
