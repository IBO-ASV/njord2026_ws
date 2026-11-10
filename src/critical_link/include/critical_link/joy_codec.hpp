#pragma once

#include <optional>
#include <vector>

#include "sensor_msgs/msg/joy.hpp"

namespace critical_link
{

constexpr size_t kMaxJoyAxes = 24;
constexpr size_t kMaxJoyButtons = 64;

std::optional<std::vector<uint8_t>> encode_joy_payload(const sensor_msgs::msg::Joy & joy);
std::optional<sensor_msgs::msg::Joy> decode_joy_payload(
  const uint8_t * data, size_t size);

}  // namespace critical_link
