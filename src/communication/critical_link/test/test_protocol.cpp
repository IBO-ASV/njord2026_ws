#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

#include "critical_link/joy_codec.hpp"
#include "critical_link/protocol.hpp"

namespace critical_link
{
namespace
{

Frame sample_frame()
{
  Frame frame;
  frame.stream = StreamId::kJoy;
  frame.flags = 3;
  frame.session_id = 0x0123456789ABCDEFULL;
  frame.sequence = 42;
  frame.source_unix_ms = 1'700'000'000'000LL;
  frame.payload = {1, 2, 3, 4, 5};
  return frame;
}

SharedKey sample_key()
{
  SharedKey key{};
  for (size_t index = 0; index < key.size(); ++index) key[index] = static_cast<uint8_t>(index);
  return key;
}

TEST(Protocol, RoundTripsFrame)
{
  const auto source = sample_frame();
  const auto key = sample_key();
  const auto bytes = encode_frame(source, key);
  ASSERT_LE(bytes.size(), 234U);
  const auto decoded = decode_frame(bytes.data(), bytes.size(), key, source.source_unix_ms);
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->stream, source.stream);
  EXPECT_EQ(decoded->flags, source.flags);
  EXPECT_EQ(decoded->session_id, source.session_id);
  EXPECT_EQ(decoded->sequence, source.sequence);
  EXPECT_EQ(decoded->source_unix_ms, source.source_unix_ms);
  EXPECT_EQ(decoded->payload, source.payload);
}

TEST(Protocol, RejectsCorruptedFrame)
{
  const auto key = sample_key();
  auto bytes = encode_frame(sample_frame(), key);
  bytes[critical_link::kHeaderSize + 1U] ^= 0x80U;
  EXPECT_FALSE(decode_frame(bytes.data(), bytes.size(), key, sample_frame().source_unix_ms).has_value());
}

TEST(Protocol, StreamDecoderResynchronizesAndHandlesFragments)
{
  const auto frame = sample_frame();
  const auto key = sample_key();
  const auto bytes = encode_frame(frame, key);
  const std::array<uint8_t, 5> garbage{{9, 8, 'N', 'C', 7}};
  FrameStreamDecoder decoder;
  EXPECT_TRUE(decoder.push(garbage.data(), garbage.size(), key, frame.source_unix_ms).empty());
  EXPECT_TRUE(decoder.push(bytes.data(), 11, key, frame.source_unix_ms).empty());
  const auto frames = decoder.push(bytes.data() + 11, bytes.size() - 11, key, frame.source_unix_ms);
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames[0].sequence, frame.sequence);
}

TEST(SequenceGate, AcceptsOnlyNewestSequenceAndRetiresOldSessions)
{
  SequenceGate gate;
  EXPECT_TRUE(gate.accept(10, StreamId::kJoy, 100));
  EXPECT_FALSE(gate.accept(10, StreamId::kJoy, 100));
  EXPECT_FALSE(gate.accept(10, StreamId::kJoy, 99));
  EXPECT_TRUE(gate.accept(10, StreamId::kJoy, 101));
  EXPECT_TRUE(gate.accept(10, StreamId::kGroundHeartbeat, 1));
  EXPECT_TRUE(gate.accept(11, StreamId::kJoy, 1));
  EXPECT_FALSE(gate.accept(10, StreamId::kJoy, 102));
}

TEST(SequenceGate, HandlesSequenceWrap)
{
  SequenceGate gate;
  EXPECT_TRUE(gate.accept(1, StreamId::kJoy, std::numeric_limits<uint32_t>::max()));
  EXPECT_TRUE(gate.accept(1, StreamId::kJoy, 0));
}

TEST(JoyCodec, RoundTripsCompactPayload)
{
  sensor_msgs::msg::Joy joy;
  joy.axes = {-1.0F, 0.25F, 1.0F};
  joy.buttons = {0, 1, 0, 1};
  const auto payload = encode_joy_payload(joy);
  ASSERT_TRUE(payload.has_value());
  ASSERT_LE(payload->size(), kMaxPayloadSize);
  const auto decoded = decode_joy_payload(payload->data(), payload->size());
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->axes, joy.axes);
  EXPECT_EQ(decoded->buttons, joy.buttons);
}

TEST(JoyCodec, RejectsNonFiniteAndMalformedValues)
{
  sensor_msgs::msg::Joy joy;
  joy.axes = {std::numeric_limits<float>::quiet_NaN()};
  EXPECT_FALSE(encode_joy_payload(joy).has_value());
  const std::array<uint8_t, 3> invalid_button{{0, 1, 2}};
  EXPECT_FALSE(decode_joy_payload(invalid_button.data(), invalid_button.size()).has_value());
}

}  // namespace
}  // namespace critical_link
