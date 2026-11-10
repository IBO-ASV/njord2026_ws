#pragma once

#include <Arduino.h>
#include <Wire.h>

class Dht20Sensor {
 public:
  static constexpr uint8_t kAddress = 0x38;

  bool begin(TwoWire &wire = Wire) {
    wire_ = &wire;
    wire_->beginTransmission(kAddress);
    if (wire_->endTransmission() != 0) return false;

    uint8_t status = 0;
    if (!readStatus(status)) return false;
    if (status & 0x08U) return true;

    if (!writeCommand(0xBE, 0x08, 0x00)) return false;
    delay(10);
    return readStatus(status) && (status & 0x08U);
  }

  bool read(float &temperature_c, float &humidity_percent) {
    if (wire_ == nullptr) return false;
    if (!writeCommand(0xAC, 0x33, 0x00)) return false;
    delay(85);

    uint8_t data[7]{};
    if (wire_->requestFrom(kAddress, static_cast<uint8_t>(sizeof(data))) !=
        sizeof(data)) {
      return false;
    }
    for (uint8_t &value : data) value = wire_->read();

    if (data[0] & 0x80U) return false;
    if (crc8(data, 6) != data[6]) return false;

    const uint32_t raw_humidity =
        (static_cast<uint32_t>(data[1]) << 12) |
        (static_cast<uint32_t>(data[2]) << 4) | (data[3] >> 4);
    const uint32_t raw_temperature =
        (static_cast<uint32_t>(data[3] & 0x0FU) << 16) |
        (static_cast<uint32_t>(data[4]) << 8) | data[5];

    humidity_percent =
        static_cast<float>(raw_humidity) * 100.0F / 1048576.0F;
    temperature_c =
        static_cast<float>(raw_temperature) * 200.0F / 1048576.0F - 50.0F;
    return isfinite(temperature_c) && temperature_c >= -40.0F &&
           temperature_c <= 85.0F && isfinite(humidity_percent) &&
           humidity_percent >= 0.0F && humidity_percent <= 100.0F;
  }

 private:
  TwoWire *wire_ = nullptr;

  bool writeCommand(uint8_t command, uint8_t arg1, uint8_t arg2) {
    wire_->beginTransmission(kAddress);
    wire_->write(command);
    wire_->write(arg1);
    wire_->write(arg2);
    return wire_->endTransmission() == 0;
  }

  bool readStatus(uint8_t &status) {
    wire_->beginTransmission(kAddress);
    wire_->write(0x71);
    if (wire_->endTransmission() != 0) return false;
    if (wire_->requestFrom(kAddress, static_cast<uint8_t>(1)) != 1) {
      return false;
    }
    status = wire_->read();
    return true;
  }

  static uint8_t crc8(const uint8_t *data, size_t length) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < length; ++i) {
      crc ^= data[i];
      for (uint8_t bit = 0; bit < 8; ++bit) {
        crc = (crc & 0x80U) ? static_cast<uint8_t>((crc << 1) ^ 0x31U)
                            : static_cast<uint8_t>(crc << 1);
      }
    }
    return crc;
  }
};
