#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace critical_link
{

struct UdpSenderSpec
{
  std::string name;
  std::string bind_address;
  std::string destination_address;
  uint16_t port{0};
};

struct UdpReceiverSpec
{
  std::string name;
  std::string bind_address;
  uint16_t port{0};
};

std::optional<UdpSenderSpec> parse_udp_sender_spec(const std::string & text);
std::optional<UdpReceiverSpec> parse_udp_receiver_spec(const std::string & text);
int open_udp_sender(const UdpSenderSpec & spec);
int open_udp_receiver(const UdpReceiverSpec & spec);
int open_serial_port(const std::string & device, int baud);
bool write_all(int fd, const uint8_t * data, size_t size);

}  // namespace critical_link
