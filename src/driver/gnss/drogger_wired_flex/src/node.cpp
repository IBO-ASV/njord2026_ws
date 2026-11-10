#include "drogger_wired_flex/node.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>

namespace drogger_wired_flex
{

DroggerWiredFlexNode::DroggerWiredFlexNode(const rclcpp::NodeOptions & options)
: Node("drogger_wired_flex", options)
{
  declare_and_get_parameters();

  fix_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>(params_.fix_topic, 10);

  io_thread_ = std::thread([this]() {io_loop();});
}

DroggerWiredFlexNode::~DroggerWiredFlexNode()
{
  keep_running_ = false;
  close_fd();

  if (io_thread_.joinable()) {
    io_thread_.join();
  }
}

void DroggerWiredFlexNode::declare_and_get_parameters()
{
  this->declare_parameter("transport", "serial");
  this->declare_parameter("serial_port", "/dev/ttyUSB0");
  this->declare_parameter("serial_baudrate", 115200);
  this->declare_parameter("tcp_host", "192.168.1.10");
  this->declare_parameter("tcp_bind_host", "0.0.0.0");
  this->declare_parameter("tcp_port", 5000);
  this->declare_parameter("udp_bind_host", "0.0.0.0");
  this->declare_parameter("udp_port", 5000);
  this->declare_parameter("frame_id", "gnss_link");
  this->declare_parameter("fix_topic", "/sensor/vehicle_gnss/fix/raw");
  this->declare_parameter("read_timeout_ms", 1000);
  this->declare_parameter("reconnect_sec", 1.0);

  params_.transport = this->get_parameter("transport").as_string();
  params_.serial_port = this->get_parameter("serial_port").as_string();
  params_.serial_baudrate = this->get_parameter("serial_baudrate").as_int();
  params_.tcp_host = this->get_parameter("tcp_host").as_string();
  params_.tcp_bind_host = this->get_parameter("tcp_bind_host").as_string();
  params_.tcp_port = this->get_parameter("tcp_port").as_int();
  params_.udp_bind_host = this->get_parameter("udp_bind_host").as_string();
  params_.udp_port = this->get_parameter("udp_port").as_int();
  params_.frame_id = this->get_parameter("frame_id").as_string();
  params_.fix_topic = this->get_parameter("fix_topic").as_string();
  params_.read_timeout_ms = this->get_parameter("read_timeout_ms").as_int();
  params_.reconnect_sec = this->get_parameter("reconnect_sec").as_double();

  RCLCPP_INFO(
    this->get_logger(),
    "transport=%s serial=%s@%d tcp=%s:%d udp_bind=%s:%d",
    params_.transport.c_str(),
    params_.serial_port.c_str(),
    params_.serial_baudrate,
    params_.tcp_host.c_str(),
    params_.tcp_port,
    params_.udp_bind_host.c_str(),
    params_.udp_port);
}

void DroggerWiredFlexNode::io_loop()
{
  while (rclcpp::ok() && keep_running_) {
    if (fd_ < 0 && !open_transport_fd()) {
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(params_.reconnect_sec));
      rclcpp::sleep_for(ns);
      continue;
    }

    struct pollfd pfd;
    pfd.fd = fd_;
    pfd.events = POLLIN;
    pfd.revents = 0;

    const int poll_ret = ::poll(&pfd, 1, params_.read_timeout_ms);
    if (poll_ret == 0) {
      continue;
    }

    if (poll_ret < 0) {
      if (errno == EINTR) {
        continue;
      }
      RCLCPP_WARN(this->get_logger(), "poll failed: %s", std::strerror(errno));
      close_fd();
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(params_.reconnect_sec));
      rclcpp::sleep_for(ns);
      continue;
    }

    if ((pfd.revents & POLLIN) == 0) {
      if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
        RCLCPP_WARN(this->get_logger(), "fd event error: revents=%d", pfd.revents);
        close_fd();
        auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(params_.reconnect_sec));
        rclcpp::sleep_for(ns);
      }
      continue;
    }

    char buf[2048];
    const ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n > 0) {
      on_bytes(buf, static_cast<std::size_t>(n));
    } else if (n == 0) {
      RCLCPP_WARN(this->get_logger(), "connection closed by peer/device");
      close_fd();
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(params_.reconnect_sec));
      rclcpp::sleep_for(ns);
    } else {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        continue;
      }
      RCLCPP_WARN(this->get_logger(), "read failed: %s", std::strerror(errno));
      close_fd();
      auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(params_.reconnect_sec));
      rclcpp::sleep_for(ns);
    }
  }
}

bool DroggerWiredFlexNode::open_transport_fd()
{
  if (params_.transport == "serial") {
    return open_serial_fd();
  }

  if (params_.transport == "tcp") {
    return open_tcp_fd();
  }

  if (params_.transport == "tcp_server") {
    return open_tcp_server_fd();
  }

  if (params_.transport == "udp") {
    return open_udp_fd();
  }

  RCLCPP_ERROR(this->get_logger(), "unknown transport: %s", params_.transport.c_str());
  return false;
}

bool DroggerWiredFlexNode::open_serial_fd()
{
  const int fd = ::open(params_.serial_port.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    RCLCPP_WARN(this->get_logger(), "open serial failed (%s): %s", params_.serial_port.c_str(),
        std::strerror(errno));
    return false;
  }

  struct termios tio;
  std::memset(&tio, 0, sizeof(tio));

  if (::tcgetattr(fd, &tio) != 0) {
    RCLCPP_WARN(this->get_logger(), "tcgetattr failed: %s", std::strerror(errno));
    ::close(fd);
    return false;
  }

  ::cfmakeraw(&tio);
  const speed_t speed = baudrate_to_speed_t(params_.serial_baudrate);
  if (::cfsetispeed(&tio, speed) != 0 || ::cfsetospeed(&tio, speed) != 0) {
    RCLCPP_WARN(this->get_logger(), "failed to set baudrate %d", params_.serial_baudrate);
    ::close(fd);
    return false;
  }

  tio.c_cflag |= (CLOCAL | CREAD);

  if (::tcsetattr(fd, TCSANOW, &tio) != 0) {
    RCLCPP_WARN(this->get_logger(), "tcsetattr failed: %s", std::strerror(errno));
    ::close(fd);
    return false;
  }

  fd_ = fd;
  RCLCPP_INFO(this->get_logger(), "serial connected: %s", params_.serial_port.c_str());
  return true;
}

bool DroggerWiredFlexNode::open_tcp_fd()
{
  struct addrinfo hints;
  std::memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  struct addrinfo * result = nullptr;
  const std::string port = std::to_string(params_.tcp_port);
  const int gai = ::getaddrinfo(params_.tcp_host.c_str(), port.c_str(), &hints, &result);
  if (gai != 0) {
    RCLCPP_WARN(this->get_logger(), "getaddrinfo failed: %s", ::gai_strerror(gai));
    return false;
  }

  int sock = -1;
  for (struct addrinfo * rp = result; rp != nullptr; rp = rp->ai_next) {
    sock = ::socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
    if (sock < 0) {
      continue;
    }

    if (::connect(sock, rp->ai_addr, rp->ai_addrlen) == 0) {
      break;
    }

    ::close(sock);
    sock = -1;
  }

  ::freeaddrinfo(result);

  if (sock < 0) {
    RCLCPP_WARN(this->get_logger(), "tcp connect failed: %s:%d", params_.tcp_host.c_str(),
        params_.tcp_port);
    return false;
  }

  const int flags = ::fcntl(sock, F_GETFL, 0);
  if (flags >= 0) {
    (void)::fcntl(sock, F_SETFL, flags | O_NONBLOCK);
  }

  fd_ = sock;
  RCLCPP_INFO(this->get_logger(), "tcp connected: %s:%d", params_.tcp_host.c_str(),
      params_.tcp_port);
  return true;
}

bool DroggerWiredFlexNode::open_tcp_server_fd()
{
  const int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (listen_fd < 0) {
    RCLCPP_WARN(this->get_logger(), "tcp server socket failed: %s", std::strerror(errno));
    return false;
  }

  const int one = 1;
  (void)::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  struct sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(params_.tcp_port));

  if (::inet_pton(AF_INET, params_.tcp_bind_host.c_str(), &addr.sin_addr) != 1) {
    RCLCPP_WARN(this->get_logger(), "invalid tcp_bind_host: %s", params_.tcp_bind_host.c_str());
    ::close(listen_fd);
    return false;
  }

  if (::bind(listen_fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) != 0) {
    RCLCPP_WARN(this->get_logger(), "tcp bind failed: %s", std::strerror(errno));
    ::close(listen_fd);
    return false;
  }

  if (::listen(listen_fd, 1) != 0) {
    RCLCPP_WARN(this->get_logger(), "tcp listen failed: %s", std::strerror(errno));
    ::close(listen_fd);
    return false;
  }

  struct pollfd pfd;
  pfd.fd = listen_fd;
  pfd.events = POLLIN;
  pfd.revents = 0;

  const int poll_ret = ::poll(&pfd, 1, params_.read_timeout_ms);
  if (poll_ret <= 0) {
    ::close(listen_fd);
    return false;
  }

  int client_fd = ::accept(listen_fd, nullptr, nullptr);
  ::close(listen_fd);
  if (client_fd < 0) {
    RCLCPP_WARN(this->get_logger(), "tcp accept failed: %s", std::strerror(errno));
    return false;
  }

  const int flags = ::fcntl(client_fd, F_GETFL, 0);
  if (flags >= 0) {
    (void)::fcntl(client_fd, F_SETFL, flags | O_NONBLOCK);
  }

  fd_ = client_fd;
  RCLCPP_INFO(
    this->get_logger(), "tcp server accepted client on %s:%d",
    params_.tcp_bind_host.c_str(), params_.tcp_port);
  return true;
}

bool DroggerWiredFlexNode::open_udp_fd()
{
  const int sock = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    RCLCPP_WARN(this->get_logger(), "udp socket failed: %s", std::strerror(errno));
    return false;
  }

  const int one = 1;
  (void)::setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

  struct sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(params_.udp_port));

  if (::inet_pton(AF_INET, params_.udp_bind_host.c_str(), &addr.sin_addr) != 1) {
    RCLCPP_WARN(this->get_logger(), "invalid udp_bind_host: %s", params_.udp_bind_host.c_str());
    ::close(sock);
    return false;
  }

  if (::bind(sock, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) != 0) {
    RCLCPP_WARN(this->get_logger(), "udp bind failed: %s", std::strerror(errno));
    ::close(sock);
    return false;
  }

  const int flags = ::fcntl(sock, F_GETFL, 0);
  if (flags >= 0) {
    (void)::fcntl(sock, F_SETFL, flags | O_NONBLOCK);
  }

  fd_ = sock;
  RCLCPP_INFO(this->get_logger(), "udp bound: %s:%d", params_.udp_bind_host.c_str(),
      params_.udp_port);
  return true;
}

void DroggerWiredFlexNode::close_fd()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void DroggerWiredFlexNode::on_bytes(const char * data, std::size_t size)
{
  line_buffer_.append(data, size);

  std::size_t line_end = line_buffer_.find('\n');
  while (line_end != std::string::npos) {
    std::string line = line_buffer_.substr(0, line_end);
    line_buffer_.erase(0, line_end + 1);

    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    process_line(line);
    line_end = line_buffer_.find('\n');
  }
}

void DroggerWiredFlexNode::process_line(std::string line)
{
  if (line.empty() || line[0] != '$') {
    return;
  }

  if (!validate_nmea_checksum(line)) {
    return;
  }

  const std::size_t star = line.find('*');
  if (star == std::string::npos || star < 2) {
    return;
  }

  const std::string payload = line.substr(1, star - 1);
  std::vector<std::string> tokens = split(payload, ',');
  if (tokens.empty()) {
    return;
  }

  if (tokens[0].size() >= 5 && tokens[0].substr(tokens[0].size() - 3) == "GGA") {
    parse_gga(tokens);
  }
}

void DroggerWiredFlexNode::parse_gga(const std::vector<std::string> & tokens)
{
  if (tokens.size() < 10) {
    return;
  }

  const std::string & lat_val = tokens[2];
  const std::string & lat_dir = tokens[3];
  const std::string & lon_val = tokens[4];
  const std::string & lon_dir = tokens[5];
  const std::string & q = tokens[6];

  if (lat_val.empty() || lat_dir.empty() || lon_val.empty() || lon_dir.empty()) {
    return;
  }

  sensor_msgs::msg::NavSatFix msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = params_.frame_id;

  try {
    msg.latitude = convert_nmea_to_latlon(lat_val, lat_dir);
    msg.longitude = convert_nmea_to_latlon(lon_val, lon_dir);

    const double altitude_msl = tokens[9].empty() ? 0.0 : std::stod(tokens[9]);
    const double geoid_sep = (tokens.size() > 11 &&
      !tokens[11].empty()) ? std::stod(tokens[11]) : 0.0;
    msg.altitude = altitude_msl + geoid_sep;

    const int quality = q.empty() ? 0 : std::stoi(q);
    msg.status.status =
      (quality >
      0) ? sensor_msgs::msg::NavSatStatus::STATUS_FIX :
      sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS |
      sensor_msgs::msg::NavSatStatus::SERVICE_GLONASS |
      sensor_msgs::msg::NavSatStatus::SERVICE_COMPASS |
      sensor_msgs::msg::NavSatStatus::SERVICE_GALILEO;

    const double hdop = (tokens.size() > 8 && !tokens[8].empty()) ? std::stod(tokens[8]) : 99.9;
    double horiz_sigma = std::max(0.5, hdop * 1.5);
    double vert_sigma = horiz_sigma * 2.0;

    if (quality == 4 || quality == 5) {
      horiz_sigma = 0.03;
      vert_sigma = 0.06;
    } else if (quality <= 0) {
      horiz_sigma = 100.0;
      vert_sigma = 200.0;
    }

    msg.position_covariance[0] = horiz_sigma * horiz_sigma;
    msg.position_covariance[4] = horiz_sigma * horiz_sigma;
    msg.position_covariance[8] = vert_sigma * vert_sigma;
    msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_APPROXIMATED;

    fix_pub_->publish(msg);
  } catch (const std::exception & e) {
    RCLCPP_WARN(this->get_logger(), "failed to parse GGA: %s", e.what());
  }
}

std::vector<std::string> DroggerWiredFlexNode::split(const std::string & s, char delimiter)
{
  std::vector<std::string> out;
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, delimiter)) {
    out.push_back(item);
  }

  if (!s.empty() && s.back() == delimiter) {
    out.emplace_back();
  }
  return out;
}

bool DroggerWiredFlexNode::validate_nmea_checksum(const std::string & sentence)
{
  const std::size_t star = sentence.find('*');
  if (star == std::string::npos || star < 2 || sentence[0] != '$') {
    return false;
  }

  unsigned int calc = 0;
  for (std::size_t i = 1; i < star; ++i) {
    calc ^= static_cast<unsigned int>(sentence[i]);
  }

  try {
    const unsigned int given = std::stoul(sentence.substr(star + 1), nullptr, 16);
    return calc == given;
  } catch (...) {
    return false;
  }
}

double DroggerWiredFlexNode::convert_nmea_to_latlon(
  const std::string & value,
  const std::string & direction)
{
  const double raw = std::stod(value);
  const int degrees = static_cast<int>(raw / 100.0);
  const double minutes = raw - (static_cast<double>(degrees) * 100.0);
  double decimal = static_cast<double>(degrees) + minutes / 60.0;

  if (direction == "S" || direction == "W") {
    decimal = -decimal;
  }
  return decimal;
}

speed_t DroggerWiredFlexNode::baudrate_to_speed_t(int baudrate)
{
  switch (baudrate) {
    case 4800:
      return B4800;
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
#ifdef B230400
    case 230400:
      return B230400;
#endif
    default:
      return B115200;
  }
}

}  // namespace drogger_wired_flex
