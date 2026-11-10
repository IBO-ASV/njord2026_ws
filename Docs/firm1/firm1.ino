#include <HardwareSerial.h>

#include <cmath>

#include "serial_protocol.hpp"
#include "thrust_duty_profile.hpp"

namespace {

constexpr uint8_t kPwmResolutionBits = 12;
constexpr uint32_t kPwmFrequencyHz = 50;
constexpr uint32_t kPwmPeriodUs = 20000;
constexpr uint16_t kEscCenterUs = 1500;
constexpr uint32_t kCommandTimeoutMs = 1000;
constexpr unsigned long kSerialBaudRate = 115200;
constexpr size_t kSerialBytesPerLoop = 128;

constexpr uint8_t kServoPins[] = {16, 4, 18, 17};
constexpr uint8_t kFetSwitchPin = 14;
constexpr uint8_t kGreenLedPin = 25;
constexpr uint8_t kYellowLedPin = 26;
constexpr uint8_t kRedLedPin = 27;
constexpr uint8_t kRelayPin = 15;

constexpr size_t kThrusterCount = 4;
constexpr uint8_t kEmergencyStopMask = 1U << 3;
constexpr uint8_t kGreenLedMask = 1U << 2;
constexpr uint8_t kYellowLedMask = 1U << 1;
constexpr uint8_t kRedLedMask = 1U;

// float32[4] + control_flags
constexpr size_t kThrusterCommandPayloadSize =
    kThrusterCount * sizeof(float) + sizeof(uint8_t);
constexpr size_t kThrusterCommandRawSize =
    serial_protocol::kHeaderSize + kThrusterCommandPayloadSize +
    serial_protocol::kCrcSize;

static_assert(sizeof(float) == 4, "Protocol requires 32-bit float");
static_assert(kThrusterCommandRawSize <= serial_protocol::kMaximumRawFrameSize,
              "Thruster command exceeds the protocol frame limit");

// Logical thrusters 1..4 are connected to servo pins 4, 1, 3, 2 respectively.
constexpr uint8_t kThrusterPins[] = {
    kServoPins[3], kServoPins[0], kServoPins[2], kServoPins[1]
};

uint8_t encodedFrame[serial_protocol::kMaximumEncodedFrameSize];
size_t encodedFrameLength = 0;
bool discardingOversizedFrame = false;
unsigned long lastValidCommandTimeMs = 0;
bool hasValidCommand = false;

void writeServoPulse(uint8_t pin, uint32_t pulseWidthUs) {
  const uint32_t duty =
      ((1U << kPwmResolutionBits) - 1U) * pulseWidthUs / kPwmPeriodUs;
  ledcWrite(pin, duty);
}

void centerAllThrusters() {
  for (const uint8_t pin : kServoPins) {
    writeServoPulse(pin, kEscCenterUs);
  }
}

void setFetEnabled(bool enabled) {
  digitalWrite(kFetSwitchPin, enabled ? HIGH : LOW);
}

void enterSafeState() {
  centerAllThrusters();
  setFetEnabled(false);
}

void configurePins() {
  for (const uint8_t pin : kServoPins) {
    ledcAttach(pin, kPwmFrequencyHz, kPwmResolutionBits);
  }

  pinMode(kFetSwitchPin, OUTPUT);
  pinMode(kGreenLedPin, OUTPUT);
  pinMode(kYellowLedPin, OUTPUT);
  pinMode(kRedLedPin, OUTPUT);
  pinMode(kRelayPin, INPUT_PULLUP);
}

bool physicalEStopActive() {
  return digitalRead(kRelayPin) == HIGH;
}

void reportRelayState() {
  Serial.write(physicalEStopActive() ? 0x01 : 0x00);
}

void updateLeds(uint8_t controlFlags) {
  digitalWrite(kGreenLedPin, (controlFlags & kGreenLedMask) != 0);
  digitalWrite(kYellowLedPin, (controlFlags & kYellowLedMask) != 0);
  digitalWrite(kRedLedPin, (controlFlags & kRedLedMask) != 0);
}

void updateThrusters(const float (&thrusts)[kThrusterCount]) {
  setFetEnabled(true);
  for (size_t i = 0; i < kThrusterCount; ++i) {
    writeServoPulse(
        kThrusterPins[i],
        thrust_profile::forceN_to_on_time(static_cast<double>(thrusts[i])));
  }
}

bool processThrusterCommand(const uint8_t* raw, size_t rawLength) {
  if (rawLength != kThrusterCommandRawSize ||
      raw[0] != serial_protocol::kVersion ||
      raw[1] != static_cast<uint8_t>(
                    serial_protocol::MessageType::kThrusterCommand) ||
      raw[4] != kThrusterCommandPayloadSize) {
    return false;
  }

  const uint16_t receivedCrc =
      serial_protocol::readUint16Le(raw + rawLength - serial_protocol::kCrcSize);
  const uint16_t calculatedCrc =
      serial_protocol::crc16CcittFalse(
          raw, rawLength - serial_protocol::kCrcSize);
  if (receivedCrc != calculatedCrc) {
    return false;
  }

  const uint8_t* payload = raw + serial_protocol::kHeaderSize;
  float thrusts[kThrusterCount];
  for (size_t i = 0; i < kThrusterCount; ++i) {
    thrusts[i] = serial_protocol::readFloat32Le(payload + i * sizeof(float));
    if (!std::isfinite(thrusts[i])) {
      return false;
    }
  }
  const uint8_t controlFlags = payload[kThrusterCount * sizeof(float)];

  lastValidCommandTimeMs = millis();
  hasValidCommand = true;
  updateLeds(controlFlags);

  if ((controlFlags & kEmergencyStopMask) != 0U) {
    enterSafeState();
  } else {
    updateThrusters(thrusts);
  }
  reportRelayState();
  return true;
}

void processEncodedFrame() {
  uint8_t raw[serial_protocol::kMaximumRawFrameSize];
  size_t rawLength = 0;
  if (serial_protocol::cobsDecode(
          encodedFrame, encodedFrameLength, raw, sizeof(raw), rawLength)) {
    processThrusterCommand(raw, rawLength);
  }
}

void processSerialInput() {
  size_t processedBytes = 0;
  while (Serial.available() > 0 && processedBytes < kSerialBytesPerLoop) {
    const int received = Serial.read();
    if (received < 0) {
      break;
    }
    ++processedBytes;
    const uint8_t byte = static_cast<uint8_t>(received);

    if (byte == 0) {
      if (discardingOversizedFrame) {
        discardingOversizedFrame = false;
      } else if (encodedFrameLength > 0) {
        processEncodedFrame();
      }
      encodedFrameLength = 0;
      continue;
    }

    if (discardingOversizedFrame) {
      continue;
    }
    if (encodedFrameLength >= sizeof(encodedFrame)) {
      encodedFrameLength = 0;
      discardingOversizedFrame = true;
      continue;
    }
    encodedFrame[encodedFrameLength++] = byte;
  }
}

void enforceSafety() {
  if (hasValidCommand &&
      millis() - lastValidCommandTimeMs > kCommandTimeoutMs) {
    enterSafeState();
  }
}

}  // namespace

void setup() {
  configurePins();
  updateLeds(0);
  enterSafeState();
  Serial.begin(kSerialBaudRate);
  delay(3000);
}

void loop() {
  enforceSafety();
  processSerialInput();
}
