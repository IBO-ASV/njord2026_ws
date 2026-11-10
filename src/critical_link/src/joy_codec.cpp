#include "critical_link/joy_codec.hpp"

#include <cmath>
#include <cstring>

namespace critical_link
{
namespace
{

void append_float(std::vector<uint8_t> & out, float value)
{
  uint32_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  for (unsigned int shift = 0; shift < 32; shift += 8) {
    out.push_back(static_cast<uint8_t>(bits >> shift));
  }
}

float read_float(const uint8_t * data)
{
  uint32_t bits = 0;
  for (unsigned int shift = 0; shift < 32; shift += 8) {
    bits |= static_cast<uint32_t>(*data++) << shift;
  }
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

}  // namespace

std::optional<std::vector<uint8_t>> encode_joy_payload(const sensor_msgs::msg::Joy & joy)
{
  if (joy.axes.size() > kMaxJoyAxes || joy.buttons.size() > kMaxJoyButtons) {
    return std::nullopt;
  }
  std::vector<uint8_t> payload;
  payload.reserve(2U + joy.axes.size() * sizeof(float) + joy.buttons.size());
  payload.push_back(static_cast<uint8_t>(joy.axes.size()));
  payload.push_back(static_cast<uint8_t>(joy.buttons.size()));
  for (const float axis : joy.axes) {
    if (!std::isfinite(axis)) {
      return std::nullopt;
    }
    append_float(payload, axis);
  }
  for (const int32_t button : joy.buttons) {
    payload.push_back(button == 0 ? 0U : 1U);
  }
  return payload;
}

std::optional<sensor_msgs::msg::Joy> decode_joy_payload(const uint8_t * data, size_t size)
{
  if (data == nullptr || size < 2U) {
    return std::nullopt;
  }
  const size_t axes_size = data[0];
  const size_t buttons_size = data[1];
  if (axes_size > kMaxJoyAxes || buttons_size > kMaxJoyButtons ||
    size != 2U + axes_size * sizeof(float) + buttons_size)
  {
    return std::nullopt;
  }

  sensor_msgs::msg::Joy joy;
  joy.axes.reserve(axes_size);
  const uint8_t * cursor = data + 2U;
  for (size_t index = 0; index < axes_size; ++index) {
    const float axis = read_float(cursor);
    if (!std::isfinite(axis)) {
      return std::nullopt;
    }
    joy.axes.push_back(axis);
    cursor += sizeof(float);
  }
  joy.buttons.reserve(buttons_size);
  for (size_t index = 0; index < buttons_size; ++index) {
    if (cursor[index] > 1U) {
      return std::nullopt;
    }
    joy.buttons.push_back(cursor[index] == 0U ? 0 : 1);
  }
  return joy;
}

}  // namespace critical_link
