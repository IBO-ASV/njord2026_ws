#include <gtest/gtest.h>

#include <unistd.h>

#include "critical_link/io.hpp"

namespace critical_link
{
namespace
{

TEST(IoConfig, ParsesSenderPath)
{
  const auto spec = parse_udp_sender_spec("usb_wifi|192.168.10.1|192.168.10.2|45100");
  ASSERT_TRUE(spec.has_value());
  EXPECT_EQ(spec->name, "usb_wifi");
  EXPECT_EQ(spec->bind_address, "192.168.10.1");
  EXPECT_EQ(spec->destination_address, "192.168.10.2");
  EXPECT_EQ(spec->port, 45100);
}

TEST(IoConfig, ParsesReceiverPath)
{
  const auto spec = parse_udp_receiver_spec("internet_vpn|10.20.0.2|45103");
  ASSERT_TRUE(spec.has_value());
  EXPECT_EQ(spec->name, "internet_vpn");
  EXPECT_EQ(spec->bind_address, "10.20.0.2");
  EXPECT_EQ(spec->port, 45103);
}

TEST(IoConfig, RejectsInvalidPaths)
{
  EXPECT_FALSE(parse_udp_sender_spec("missing-fields").has_value());
  EXPECT_FALSE(parse_udp_sender_spec("x|127.0.0.1|127.0.0.1|70000").has_value());
  EXPECT_FALSE(parse_udp_receiver_spec("x|127.0.0.1|0").has_value());
}

TEST(IoConfig, ResolvesHostnameForUdp)
{
  const UdpSenderSpec spec{"loopback", "localhost", "localhost", 45100};
  const int fd = open_udp_sender(spec);
  ASSERT_GE(fd, 0);
  close(fd);
}

}  // namespace
}  // namespace critical_link
