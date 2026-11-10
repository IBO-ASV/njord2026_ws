#ifndef SIMPLE_MANUAL__SBUS_JOY_HPP_
#define SIMPLE_MANUAL__SBUS_JOY_HPP_

#include <array>
#include <string_view>

#include "sensor_msgs/msg/joy.hpp"

namespace simple_manual
{

struct SbusFrame
{
  std::array<int, 8> channels{};
  bool lost_frame{false};
  bool failsafe{false};
};

bool parse_sbus_line(std::string_view line, SbusFrame & frame);
sensor_msgs::msg::Joy sbus_to_joy(
  const SbusFrame & frame, int minimum, int center, int maximum, int button_threshold,
  int failsafe_button);

}  // namespace simple_manual

#endif  // SIMPLE_MANUAL__SBUS_JOY_HPP_
