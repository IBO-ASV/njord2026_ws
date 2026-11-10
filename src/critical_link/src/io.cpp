#include "critical_link/io.hpp"

#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <charconv>
#include <sstream>
#include <vector>

namespace critical_link
{
namespace
{

std::vector<std::string> split(const std::string & text, char delimiter)
{
  std::vector<std::string> fields;
  std::stringstream stream(text);
  std::string field;
  while (std::getline(stream, field, delimiter)) {
    fields.push_back(field);
  }
  return fields;
}

std::optional<uint16_t> parse_port(const std::string & text)
{
  unsigned int value = 0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size() ||
    value == 0U || value > 65535U)
  {
    return std::nullopt;
  }
  return static_cast<uint16_t>(value);
}

bool fill_address(const std::string & address, uint16_t port, sockaddr_in & output)
{
  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  addrinfo * results = nullptr;
  if (getaddrinfo(address.c_str(), nullptr, &hints, &results) != 0) {
    return false;
  }
  output = *reinterpret_cast<const sockaddr_in *>(results->ai_addr);
  output.sin_port = htons(port);
  freeaddrinfo(results);
  return true;
}

std::optional<speed_t> baud_flag(int baud)
{
  switch (baud) {
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return std::nullopt;
  }
}

}  // namespace

std::optional<UdpSenderSpec> parse_udp_sender_spec(const std::string & text)
{
  const auto fields = split(text, '|');
  if (fields.size() != 4U || fields[0].empty() || fields[1].empty() || fields[2].empty()) {
    return std::nullopt;
  }
  const auto port = parse_port(fields[3]);
  if (!port) {
    return std::nullopt;
  }
  return UdpSenderSpec{fields[0], fields[1], fields[2], *port};
}

std::optional<UdpReceiverSpec> parse_udp_receiver_spec(const std::string & text)
{
  const auto fields = split(text, '|');
  if (fields.size() != 3U || fields[0].empty() || fields[1].empty()) {
    return std::nullopt;
  }
  const auto port = parse_port(fields[2]);
  if (!port) {
    return std::nullopt;
  }
  return UdpReceiverSpec{fields[0], fields[1], *port};
}

int open_udp_sender(const UdpSenderSpec & spec)
{
  sockaddr_in bind_address{};
  if (!fill_address(spec.bind_address, 0, bind_address)) {
    errno = EINVAL;
    return -1;
  }
  sockaddr_in destination{};
  if (!fill_address(spec.destination_address, spec.port, destination)) {
    errno = EINVAL;
    return -1;
  }

  const int fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    return -1;
  }
  if (bind(fd, reinterpret_cast<const sockaddr *>(&bind_address), sizeof(bind_address)) < 0 ||
    connect(fd, reinterpret_cast<const sockaddr *>(&destination), sizeof(destination)) < 0)
  {
    const int saved_errno = errno;
    close(fd);
    errno = saved_errno;
    return -1;
  }
  return fd;
}

int open_udp_receiver(const UdpReceiverSpec & spec)
{
  sockaddr_in bind_address{};
  if (!fill_address(spec.bind_address, spec.port, bind_address)) {
    errno = EINVAL;
    return -1;
  }
  const int fd = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (fd < 0) {
    return -1;
  }
  const int reuse = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  timeval timeout{};
  timeout.tv_usec = 200000;
  setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
  if (bind(fd, reinterpret_cast<const sockaddr *>(&bind_address), sizeof(bind_address)) < 0) {
    const int saved_errno = errno;
    close(fd);
    errno = saved_errno;
    return -1;
  }
  return fd;
}

int open_serial_port(const std::string & device, int baud)
{
  const auto speed = baud_flag(baud);
  if (!speed) {
    errno = EINVAL;
    return -1;
  }
  const int fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) {
    return -1;
  }
  termios options{};
  if (tcgetattr(fd, &options) != 0) {
    const int saved_errno = errno;
    close(fd);
    errno = saved_errno;
    return -1;
  }
  cfmakeraw(&options);
  cfsetispeed(&options, *speed);
  cfsetospeed(&options, *speed);
  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CRTSCTS;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &options) != 0) {
    const int saved_errno = errno;
    close(fd);
    errno = saved_errno;
    return -1;
  }
  tcflush(fd, TCIOFLUSH);
  return fd;
}

bool write_all(int fd, const uint8_t * data, size_t size)
{
  size_t written = 0;
  int would_block_count = 0;
  while (written < size) {
    const ssize_t result = write(fd, data + written, size - written);
    if (result > 0) {
      written += static_cast<size_t>(result);
      continue;
    }
    if (result < 0 && errno == EINTR) {
      continue;
    }
    if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      if (++would_block_count > 20) {
        errno = EAGAIN;
        return false;
      }
      usleep(500);
      continue;
    }
    return false;
  }
  return true;
}

}  // namespace critical_link
