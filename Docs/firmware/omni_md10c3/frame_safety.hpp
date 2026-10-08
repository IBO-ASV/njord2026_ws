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

inline bool mustDiscardPartialFrame(bool command_timed_out, bool has_partial_frame)
{
  return command_timed_out && has_partial_frame;
}

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
