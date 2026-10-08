#include <HardwareSerial.h>

#include <cmath>

#include "serial_protocol.hpp"

namespace {

// This is a dedicated MD10C sketch.  It is intentionally not a replacement
// for Docs/firm1/firm1.ino, which controls the watercraft's four ESCs.
constexpr uint8_t kPwmPins[] = {16, 4, 18};       // PWM1, PWM2, PWM3
constexpr uint8_t kDirectionPins[] = {25, 26, 27};  // DIR1, DIR2, DIR3
constexpr size_t kMotorCount = 3;
constexpr uint32_t kPwmFrequencyHz = 20000;
// 20 kHz is within the MD10C's specified PWM range.  At the ESP32 80 MHz
// APB LEDC source, 12-bit would require 81.92 MHz and cannot be configured;
// 11-bit needs 40.96 MHz and has margin for a valid LEDC divider.
constexpr uint8_t kPwmResolutionBits = 11;
constexpr uint32_t kCommandTimeoutMs = 250;
constexpr uint32_t kFrameInterByteTimeoutMs = 50;
constexpr uint32_t kDirectionChangeDeadtimeUs = 2000;
constexpr unsigned long kSerialBaudRate = 115200;
constexpr size_t kSerialBytesPerLoop = 128;
constexpr float kDutyLimit = 0.50F;
constexpr float kCalibrationDutyLimit = 0.15F;
constexpr uint32_t kCalibrationPulseLimitMs = 1000;
constexpr uint8_t kEmergencyStopMask = 1U << 3;

// SAFETY INTERLOCK: retain false until the complete low-duty polarity test is
// signed off.  Releasing this firmware interlock is necessary but not
// sufficient: the host YAML has a second confirmation interlock.
constexpr bool kMotorOutputEnabled = false;

// Calibration is a distinct, deliberately limited mode.  It may never be
// compiled together with normal output, and remains off by default.
constexpr bool kCalibrationOutputEnabled = false;
static_assert(!(kMotorOutputEnabled && kCalibrationOutputEnabled),
  "Normal and calibration motor output modes are mutually exclusive");

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
unsigned long last_partial_frame_byte_time_ms = 0;
bool has_valid_command = false;
bool has_partial_frame = false;
bool pwm_hardware_ready = false;
bool pwm_hardware_initialization_failed = false;
bool pwm_attached[kMotorCount] = {false, false, false};
bool last_direction_high[kMotorCount] = {false, false, false};
bool calibration_pulse_active = false;
bool calibration_pulse_latched = false;
unsigned long calibration_pulse_start_time_ms = 0;

// Keep the sketch compatible with the same Arduino ESP32 toolchain family as
// the existing watercraft firmware.  std::clamp is C++17-only and is not
// available in every supported Arduino core toolchain.
float clampDuty(float duty, float limit = kDutyLimit)
{
  if (duty > limit) {
    return limit;
  }
  if (duty < -limit) {
    return -limit;
  }
  return duty;
}

uint32_t dutyToPwmCount(float duty, float limit)
{
  const float magnitude = std::fabs(duty);
  const float limited = magnitude > limit ? limit : magnitude;
  const uint32_t maximum = (1U << kPwmResolutionBits) - 1U;
  return static_cast<uint32_t>(std::lround(limited * static_cast<float>(maximum)));
}

void forcePinsLowAndDetachPwm()
{
  for (size_t i = 0; i < kMotorCount; ++i) {
    if (pwm_attached[i]) {
      ledcWrite(kPwmPins[i], 0);
      ledcDetach(kPwmPins[i]);
      pwm_attached[i] = false;
    }
    pinMode(kPwmPins[i], OUTPUT);
    digitalWrite(kPwmPins[i], LOW);
    pinMode(kDirectionPins[i], OUTPUT);
    digitalWrite(kDirectionPins[i], LOW);
    last_direction_high[i] = false;
  }
}

void enterSafeState()
{
  for (size_t i = 0; i < kMotorCount; ++i) {
    if (pwm_attached[i]) {
      ledcWrite(kPwmPins[i], 0);
    } else {
      pinMode(kPwmPins[i], OUTPUT);
      digitalWrite(kPwmPins[i], LOW);
    }
    digitalWrite(kDirectionPins[i], LOW);
    last_direction_high[i] = false;
  }
}

bool configurePins()
{
  for (size_t i = 0; i < kMotorCount; ++i) {
    pinMode(kPwmPins[i], OUTPUT);
    digitalWrite(kPwmPins[i], LOW);
    pinMode(kDirectionPins[i], OUTPUT);
    digitalWrite(kDirectionPins[i], LOW);
  }
  for (size_t i = 0; i < kMotorCount; ++i) {
    if (!ledcAttach(kPwmPins[i], kPwmFrequencyHz, kPwmResolutionBits)) {
      pwm_hardware_initialization_failed = true;
      forcePinsLowAndDetachPwm();
      return false;
    }
    pwm_attached[i] = true;
    if (!ledcWrite(kPwmPins[i], 0)) {
      pwm_hardware_initialization_failed = true;
      forcePinsLowAndDetachPwm();
      return false;
    }
  }
  return true;
}

void applyDuty(const float (&duty)[kMotorCount])
{
  if (!pwm_hardware_ready || pwm_hardware_initialization_failed ||
    (!kMotorOutputEnabled && !kCalibrationOutputEnabled))
  {
    enterSafeState();
    return;
  }

  size_t active_motor_count = 0;
  for (size_t i = 0; i < kMotorCount; ++i) {
    if (std::fabs(duty[i]) > 0.0F) {
      ++active_motor_count;
    }
  }

  if (kCalibrationOutputEnabled) {
    // Calibration only permits one wheel and a bounded pulse.  A zero or
    // emergency frame is required before the next pulse can begin.
    if (active_motor_count == 0U) {
      calibration_pulse_active = false;
      calibration_pulse_latched = false;
      enterSafeState();
      return;
    }
    if (active_motor_count != 1U || calibration_pulse_latched) {
      enterSafeState();
      return;
    }
    const unsigned long now = millis();
    if (!calibration_pulse_active) {
      calibration_pulse_active = true;
      calibration_pulse_start_time_ms = now;
    } else if (now - calibration_pulse_start_time_ms >= kCalibrationPulseLimitMs) {
      calibration_pulse_latched = true;
      enterSafeState();
      return;
    }
  }

  const float active_duty_limit = kCalibrationOutputEnabled ?
    kCalibrationDutyLimit : kDutyLimit;
  for (size_t i = 0; i < kMotorCount; ++i) {
    const float clamped = clampDuty(duty[i], active_duty_limit);
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
    if (!ledcWrite(kPwmPins[i], dutyToPwmCount(clamped, active_duty_limit))) {
      pwm_hardware_initialization_failed = true;
      forcePinsLowAndDetachPwm();
      return;
    }
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
    duty[i] = clampDuty(duty[i]);
  }
  const uint8_t flags = payload[kMotorCount * sizeof(float)];

  has_valid_command = true;
  last_valid_command_time_ms = millis();
  if ((flags & kEmergencyStopMask) != 0U) {
    calibration_pulse_active = false;
    calibration_pulse_latched = false;
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
      has_partial_frame = false;
      continue;
    }
    if (discarding_oversized_frame) {
      continue;
    }
    if (encoded_frame_length >= sizeof(encoded_frame)) {
      encoded_frame_length = 0;
      discarding_oversized_frame = true;
      has_partial_frame = false;
      enterSafeState();
      continue;
    }
    encoded_frame[encoded_frame_length++] = byte;
    has_partial_frame = true;
    last_partial_frame_byte_time_ms = millis();
  }
}

void enforceSafety()
{
  if (has_valid_command && millis() - last_valid_command_time_ms > kCommandTimeoutMs) {
    enterSafeState();
  }
  if (has_partial_frame &&
    millis() - last_partial_frame_byte_time_ms > kFrameInterByteTimeoutMs)
  {
    // Do not accept a delayed suffix as the end of a frame that began before
    // the communication watchdog expired.  Resynchronize only after its next
    // delimiter, while keeping every output low.
    encoded_frame_length = 0;
    has_partial_frame = false;
    discarding_oversized_frame = true;
    enterSafeState();
  }
}

}  // namespace

void setup()
{
  pwm_hardware_ready = configurePins();
  enterSafeState();
  Serial.begin(kSerialBaudRate);
}

void loop()
{
  enforceSafety();
  processSerialInput();
}
