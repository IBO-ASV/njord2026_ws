#pragma once

#include <Arduino.h>

namespace bms {

constexpr uint32_t kPacketMagic = 0x424D5334UL;  // "BMS4"
constexpr uint8_t kProtocolVersion = 2;
constexpr uint8_t kCellCount = 4;

enum PacketFlags : uint16_t {
  kAdcSaturated = 1U << 0,
  kTemperatureValid = 1U << 1,
  kDht20Error = 1U << 2,
};

struct __attribute__((packed)) CellTelemetry {
  uint32_t magic;
  uint8_t protocol_version;
  uint8_t cell_id;
  uint16_t flags;
  uint32_t sequence;
  uint32_t uptime_ms;
  uint16_t raw_adc;
  uint16_t sample_count;
  float adc_voltage_v;
  float cell_voltage_v;
  float temperature_c;
  float humidity_percent;
};

static_assert(sizeof(CellTelemetry) == 36, "Unexpected packet layout");

inline bool isValid(const CellTelemetry &packet) {
  return packet.magic == kPacketMagic &&
         packet.protocol_version == kProtocolVersion &&
         packet.cell_id >= 1 && packet.cell_id <= kCellCount &&
         isfinite(packet.cell_voltage_v) && packet.cell_voltage_v >= 0.0F &&
         packet.cell_voltage_v <= 6.0F &&
         (!(packet.flags & kTemperatureValid) ||
          (isfinite(packet.temperature_c) && packet.temperature_c >= -40.0F &&
           packet.temperature_c <= 85.0F &&
           isfinite(packet.humidity_percent) &&
           packet.humidity_percent >= 0.0F &&
           packet.humidity_percent <= 100.0F));
}

}  // namespace bms
