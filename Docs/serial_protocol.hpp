#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace serial_protocol {

constexpr uint8_t kVersion = 0x01;
constexpr size_t kHeaderSize = 5;
constexpr size_t kCrcSize = 2;
constexpr size_t kMinimumFrameSize = kHeaderSize + kCrcSize;
constexpr size_t kMaximumRawFrameSize = 64;
constexpr size_t kMaximumEncodedFrameSize = 65;

enum class MessageType : uint8_t {
  kThrusterCommand = 0x01,
};

inline uint16_t readUint16Le(const uint8_t* bytes) {
  return static_cast<uint16_t>(bytes[0]) |
         (static_cast<uint16_t>(bytes[1]) << 8U);
}

inline uint32_t readUint32Le(const uint8_t* bytes) {
  return static_cast<uint32_t>(bytes[0]) |
         (static_cast<uint32_t>(bytes[1]) << 8U) |
         (static_cast<uint32_t>(bytes[2]) << 16U) |
         (static_cast<uint32_t>(bytes[3]) << 24U);
}

inline float readFloat32Le(const uint8_t* bytes) {
  const uint32_t bits = readUint32Le(bytes);
  float value;
  static_assert(sizeof(value) == sizeof(bits), "Protocol requires 32-bit float");
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

// CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, refin=false,
// refout=false, xorout=0x0000.
constexpr uint16_t crc16CcittFalse(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFFU;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8U;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0U
                ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U)
                : static_cast<uint16_t>(crc << 1U);
    }
  }
  return crc;
}

inline bool cobsDecode(const uint8_t* encoded, size_t encodedLength,
                       uint8_t* decoded, size_t decodedCapacity,
                       size_t& decodedLength) {
  decodedLength = 0;
  if (encodedLength == 0) {
    return false;
  }

  size_t readIndex = 0;
  while (readIndex < encodedLength) {
    const uint8_t code = encoded[readIndex++];
    if (code == 0) {
      return false;
    }
    const size_t bytesToCopy = static_cast<size_t>(code) - 1U;
    if (bytesToCopy > encodedLength - readIndex ||
        bytesToCopy > decodedCapacity - decodedLength) {
      return false;
    }
    for (size_t i = 0; i < bytesToCopy; ++i) {
      decoded[decodedLength++] = encoded[readIndex++];
    }
    if (code != 0xFFU && readIndex < encodedLength) {
      if (decodedLength >= decodedCapacity) {
        return false;
      }
      decoded[decodedLength++] = 0;
    }
  }
  return true;
}

}  // namespace serial_protocol
