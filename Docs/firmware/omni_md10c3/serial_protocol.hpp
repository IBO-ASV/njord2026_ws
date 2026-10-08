#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace serial_protocol
{

constexpr uint8_t kVersion = 0x01;
constexpr size_t kHeaderSize = 5;
constexpr size_t kCrcSize = 2;
constexpr size_t kMaximumRawFrameSize = 64;
constexpr size_t kMaximumEncodedFrameSize = 65;

enum class MessageType : uint8_t {
  kOmniMd10c3Duty = 0x02,
};

inline uint16_t readUint16Le(const uint8_t * bytes)
{
  return static_cast<uint16_t>(bytes[0]) |
         (static_cast<uint16_t>(bytes[1]) << 8U);
}

inline uint32_t readUint32Le(const uint8_t * bytes)
{
  return static_cast<uint32_t>(bytes[0]) |
         (static_cast<uint32_t>(bytes[1]) << 8U) |
         (static_cast<uint32_t>(bytes[2]) << 16U) |
         (static_cast<uint32_t>(bytes[3]) << 24U);
}

inline float readFloat32Le(const uint8_t * bytes)
{
  const uint32_t bits = readUint32Le(bytes);
  float value;
  static_assert(sizeof(value) == sizeof(bits), "Protocol requires 32-bit float");
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

constexpr uint16_t crc16CcittFalse(const uint8_t * data, size_t length)
{
  uint16_t crc = 0xFFFFU;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8U;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0U ?
        static_cast<uint16_t>((crc << 1U) ^ 0x1021U) :
        static_cast<uint16_t>(crc << 1U);
    }
  }
  return crc;
}

inline bool cobsDecode(
  const uint8_t * encoded, size_t encoded_length, uint8_t * decoded,
  size_t decoded_capacity, size_t & decoded_length)
{
  decoded_length = 0;
  if (encoded_length == 0) {
    return false;
  }

  size_t read_index = 0;
  while (read_index < encoded_length) {
    const uint8_t code = encoded[read_index++];
    if (code == 0) {
      return false;
    }
    const size_t bytes_to_copy = static_cast<size_t>(code) - 1U;
    if (bytes_to_copy > encoded_length - read_index ||
      bytes_to_copy > decoded_capacity - decoded_length)
    {
      return false;
    }
    for (size_t i = 0; i < bytes_to_copy; ++i) {
      decoded[decoded_length++] = encoded[read_index++];
    }
    if (code != 0xFFU && read_index < encoded_length) {
      if (decoded_length >= decoded_capacity) {
        return false;
      }
      decoded[decoded_length++] = 0;
    }
  }
  return true;
}

}  // namespace serial_protocol
