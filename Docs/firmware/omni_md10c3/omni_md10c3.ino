#include <HardwareSerial.h>

#include <algorithm>
#include <cmath>

#include "serial_protocol.hpp"

namespace {

// This is a dedicated MD10C sketch.  It is intentionally not a replacement
// for Docs/firm1/firm1.ino, which controls the watercraft's four ESCs.
constexpr uint8_t kPwmPins[] = {16, 4, 18};       // PWM1, PWM2, PWM3
constexpr uint8_t kDirectionPins[] = {25, 26, 27};  // DIR1, DIR2, DIR3
constexpr size_t kMotorCount = 3;
constexpr uint32_t kPwmFrequencyHz = 20000;
constexpr uint8_t kPwmResolutionBits = 12;
constexpr uint32_t kCommandTimeoutMs = 250;
constexpr uint32_t kDirectionChangeDeadtimeUs = 2000;
constexpr unsigned long kSerialBaudRate = 115200;
constexpr size_t kSerialBytesPerLoop = 128;
constexpr float kDutyLimit = 0.50F;
constexpr uint8_t kEmergencyStopMask = 1U << 3;

// SAFETY INTERLOCK: retain false until the complete low-duty polarity test is
// signed off.  Releasing this firmware interlock is necessary but not
// sufficient: the host YAML has a second confirmation interlock.
constexpr bool kMotorOutputEnabled = false;

// Positive tangential wheel direction is an assumption.  Calibrate each
// value separately with the wheels clear of the floor.  Do not infer this
// mapping from an MD10C terminal label or the motor wire colours.
constexpr bool kDirectionInverted[] = {false, false, false};

// float32[3] signed duty ratios + control flags.
constexpr size_t kPayloadSize = kMotorCount * sizeof(float) + sizeof(uint8_t);
constexpr size_t kRawFrameSize =
  serial_protocol::kHeaderSize + kPayloadSize + serial_protocol::kCrcSize;
static_assert(sizeof(float) == 4, "Protocol requires 32-bit float");
static_assert(kRawFrameSize <= serial_protocol::kMaximumRawFrameSize,
  "Command exceeds protocol frame limit");

uint8_t encoded_frame[serial_protocol::kMaximumEncodedFrameSize];
size_t encoded_frame_length = 0;
bool discarding_oversized_frame = false;
unsigned long last_valid_command_time_ms = 0;
bool has_valid_command = false;
bool last_direction_high[kMotorCount] = {false, false, false};

uint32_t dutyToPwmCount(float duty)
{
  const float limited = std::clamp(std::fabs(duty), 0.0F, kDutyLimit);
  const uint32_t maximum = (1U << kPwmResolutionBits) - 1U;
  return static_cast<uint32_t>(std::lround(limited * static_cast<float>(maximum)));
}

void enterSafeState()
{
  for (size_t i = 0; i < kMotorCount; ++i) {
    ledcWrite(kPwmPins[i], 0);
    digitalWrite(kDirectionPins[i], LOW);
    last_direction_high[i] = false;
  }
}

void configurePins()
{
  for (size_t i = 0; i < kMotorCount; ++i) {
    ledcAttach(kPwmPins[i], kPwmFrequencyHz, kPwmResolutionBits);
    pinMode(kDirectionPins[i], OUTPUT);
  }
}

void applyDuty(const float (&duty)[kMotorCount])
{
  if (!kMotorOutputEnabled) {
    enterSafeState();
    return;
  }
  for (size_t i = 0; i < kMotorCount; ++i) {
    const float clamped = std::clamp(duty[i], -kDutyLimit, kDutyLimit);
    const bool forward = clamped >= 0.0F;
    const bool direction_high = forward ^ kDirectionInverted[i];
    if (direction_high != last_direction_high[i]) {
      // Do not reverse an energized MD10C channel in one write.  The bridge is
      // de-energized before its DIR transition even for a valid sign change.
      ledcWrite(kPwmPins[i], 0);
      delayMicroseconds(kDirectionChangeDeadtimeUs);
      last_direction_high[i] = direction_high;
    }
    digitalWrite(kDirectionPins[i], direction_high ? HIGH : LOW);
    ledcWrite(kPwmPins[i], dutyToPwmCount(clamped));
  }
}

void reportStatus()
{
  // Maintains a one-byte reply compatible with the serial reader.  This board
  // has no verified independent relay feedback on the MD10C profile, so bit0
  // is always zero and must not be interpreted as a physical E-stop state.
  Serial.write(0x00);
}

bool processCommand(const uint8_t * raw, size_t raw_length)
{
  if (raw_length != kRawFrameSize || raw[0] != serial_protocol::kVersion ||
    raw[1] != static_cast<uint8_t>(serial_protocol::MessageType::kOmniMd10c3Duty) ||
    raw[4] != kPayloadSize)
  {
    return false;
  }

  const uint16_t received_crc = serial_protocol::readUint16Le(
    raw + raw_length - serial_protocol::kCrcSize);
  const uint16_t calculated_crc = serial_protocol::crc16CcittFalse(
    raw, raw_length - serial_protocol::kCrcSize);
  if (received_crc != calculated_crc) {
    return false;
  }

  const uint8_t * payload = raw + serial_protocol::kHeaderSize;
  float duty[kMotorCount];
  for (size_t i = 0; i < kMotorCount; ++i) {
    duty[i] = serial_protocol::readFloat32Le(payload + i * sizeof(float));
    if (!std::isfinite(duty[i])) {
      return false;
    }
    // The firmware owns a second, non-bypassable duty boundary even if a
    // faulty or older host sends a larger finite value.
    duty[i] = std::clamp(duty[i], -kDutyLimit, kDutyLimit);
  }
  const uint8_t flags = payload[kMotorCount * sizeof(float)];

  has_valid_command = true;
  last_valid_command_time_ms = millis();
  if ((flags & kEmergencyStopMask) != 0U) {
    enterSafeState();
  } else {
    applyDuty(duty);
  }
  reportStatus();
  return true;
}

void processEncodedFrame()
{
  uint8_t raw[serial_protocol::kMaximumRawFrameSize];
  size_t raw_length = 0;
  const bool decoded = serial_protocol::cobsDecode(
    encoded_frame, encoded_frame_length, raw, sizeof(raw), raw_length);
  if (!decoded || !processCommand(raw, raw_length)) {
    // A malformed or non-finite command is a stop condition, not merely a
    // dropped update.  It deliberately does not refresh the communication
    // watchdog, so continued corruption remains stopped.
    enterSafeState();
  }
}

void processSerialInput()
{
  size_t processed_bytes = 0;
  while (Serial.available() > 0 && processed_bytes < kSerialBytesPerLoop) {
    const int received = Serial.read();
    if (received < 0) {
      break;
    }
    ++processed_bytes;
    const uint8_t byte = static_cast<uint8_t>(received);
    if (byte == 0) {
      if (discarding_oversized_frame) {
        discarding_oversized_frame = false;
      } else if (encoded_frame_length > 0) {
        processEncodedFrame();
      }
      encoded_frame_length = 0;
      continue;
    }
    if (discarding_oversized_frame) {
      continue;
    }
    if (encoded_frame_length >= sizeof(encoded_frame)) {
      encoded_frame_length = 0;
      discarding_oversized_frame = true;
      enterSafeState();
      continue;
    }
    encoded_frame[encoded_frame_length++] = byte;
  }
}

void enforceSafety()
{
  if (has_valid_command && millis() - last_valid_command_time_ms > kCommandTimeoutMs) {
    enterSafeState();
  }
}

}  // namespace

void setup()
{
  configurePins();
  enterSafeState();
  Serial.begin(kSerialBaudRate);
}

void loop()
{
  enforceSafety();
  processSerialInput();
}
