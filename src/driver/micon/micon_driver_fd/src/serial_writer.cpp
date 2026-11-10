#include "micon_driver_fd/serial_writer.hpp"

#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <functional>
#include <limits>
#include <termios.h>
#include <unistd.h>

namespace micon_driver_fd
{

namespace
{

void append_uint16_le(Packet & packet, uint16_t value)
{
  packet.push_back(static_cast<uint8_t>(value & 0xFFu));
  packet.push_back(static_cast<uint8_t>((value >> 8u) & 0xFFu));
}

void append_uint32_le(Packet & packet, uint32_t value)
{
  packet.push_back(static_cast<uint8_t>(value & 0xFFu));
  packet.push_back(static_cast<uint8_t>((value >> 8u) & 0xFFu));
  packet.push_back(static_cast<uint8_t>((value >> 16u) & 0xFFu));
  packet.push_back(static_cast<uint8_t>((value >> 24u) & 0xFFu));
}

void append_float32_le(Packet & packet, float value)
{
  uint32_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "Protocol requires 32-bit float");
  std::memcpy(&bits, &value, sizeof(bits));
  append_uint32_le(packet, bits);
}

uint16_t crc16_ccitt_false(const uint8_t * data, size_t length)
{
  uint16_t crc = 0xFFFFu;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8u;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000u) != 0u ?
        static_cast<uint16_t>((crc << 1u) ^ 0x1021u) :
        static_cast<uint16_t>(crc << 1u);
    }
  }
  return crc;
}

Packet cobs_encode(const Packet & raw)
{
  Packet encoded;
  encoded.reserve(raw.size() + 1);

  size_t code_index = 0;
  uint8_t code = 1;
  encoded.push_back(0);

  for (const uint8_t byte : raw) {
    if (byte == 0) {
      encoded[code_index] = code;
      code_index = encoded.size();
      encoded.push_back(0);
      code = 1;
      continue;
    }

    encoded.push_back(byte);
    ++code;
    if (code == std::numeric_limits<uint8_t>::max()) {
      encoded[code_index] = code;
      code_index = encoded.size();
      encoded.push_back(0);
      code = 1;
    }
  }

  encoded[code_index] = code;
  return encoded;
}

}  // namespace

Packet encode_packet(
  const std::array<float, 4> & thrust,
  const Flags & flags,
  uint16_t sequence)
{
  Packet raw;
  raw.reserve(kRawFrameSize);
  raw.push_back(kProtocolVersion);
  raw.push_back(kThrusterCommandType);
  append_uint16_le(raw, sequence);
  raw.push_back(static_cast<uint8_t>(kPayloadSize));

  for (size_t i = 0; i < thrust.size(); ++i) {
    append_float32_le(raw, thrust[i]);
  }
  uint8_t control_flags = 0;
  if (flags.emergency) {control_flags |= (1u << 3);}
  if (flags.green) {control_flags |= (1u << 2);}
  if (flags.yellow) {control_flags |= (1u << 1);}
  if (flags.red) {control_flags |= (1u << 0);}
  raw.push_back(control_flags);

  const uint16_t crc = crc16_ccitt_false(raw.data(), raw.size());
  append_uint16_le(raw, crc);

  Packet packet = cobs_encode(raw);
  packet.push_back(0);
  return packet;
}

SerialWriter::SerialWriter(const rclcpp::NodeOptions & options)
: Node("serial_writer", options)
{
  serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  baud_ = declare_parameter<int>("baud", 115200);
  command_topic_ = declare_parameter<std::string>("command_topic", "/thruster_command");
  ground_station_heartbeat_topic_ = declare_parameter<std::string>(
    "ground_station_heartbeat_topic", "/heartbeat/ground_station");
  ground_station_heartbeat_timeout_sec_ = declare_parameter<double>(
    "ground_station_heartbeat_timeout_sec", 0.0);

  sub_thrust_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    command_topic_, 10, std::bind(&SerialWriter::thrust_cb, this, std::placeholders::_1));
  sub_soft_emg_ = create_subscription<std_msgs::msg::Bool>(
    "/soft_emg", 10, std::bind(&SerialWriter::soft_emg_cb, this, std::placeholders::_1));
  sub_red_ = create_subscription<std_msgs::msg::Bool>(
    "/red", 10, std::bind(&SerialWriter::red_cb, this, std::placeholders::_1));
  sub_yellow_ = create_subscription<std_msgs::msg::Bool>(
    "/yellow", 10, std::bind(&SerialWriter::yellow_cb, this, std::placeholders::_1));
  sub_green_ = create_subscription<std_msgs::msg::Bool>(
    "/green", 10, std::bind(&SerialWriter::green_cb, this, std::placeholders::_1));
  if (ground_station_heartbeat_timeout_sec_ > 0.0) {
    sub_ground_station_heartbeat_ = create_subscription<std_msgs::msg::Empty>(
      ground_station_heartbeat_topic_, 10,
      std::bind(&SerialWriter::ground_station_heartbeat_cb, this, std::placeholders::_1));
    sub_sbus_command_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_sbus", 10,
      std::bind(&SerialWriter::sbus_command_cb, this, std::placeholders::_1));
  }
  pub_relay_active_ = create_publisher<std_msgs::msg::Bool>("/micon/relay_active", 10);
  pub_safety_emergency_ = create_publisher<std_msgs::msg::UInt8>(
    "/safety/emergency_stop", rclcpp::QoS(1).transient_local());

  fd_ = open_serial(serial_port_, baud_);
  timer_ = create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&SerialWriter::timer_cb, this));
  RCLCPP_INFO(get_logger(), "serial_writer started, port: %s", serial_port_.c_str());
}

SerialWriter::~SerialWriter()
{
  if (fd_ < 0) {
    return;
  }

  // Send an explicit final stop command before closing the port.  This makes
  // shutdown safe even if the ESP32 has not reached its communication timeout
  // yet, and leaves the warning LEDs in the commanded emergency/red state.
  const std::array<float, 4> zero_thrust{{0.0F, 0.0F, 0.0F, 0.0F}};
  Flags shutdown_flags;
  shutdown_flags.emergency = true;
  shutdown_flags.red = true;
  const Packet packet = encode_packet(zero_thrust, shutdown_flags, sequence_++);

  size_t written = 0;
  for (int attempt = 0; written < packet.size() && attempt < 100; ++attempt) {
    const ssize_t result = write(
      fd_, packet.data() + written, packet.size() - written);
    if (result > 0) {
      written += static_cast<size_t>(result);
    } else if (result < 0 && (errno == EINTR)) {
      continue;
    } else if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      usleep(1000);
    } else {
      break;
    }
  }
  if (written == packet.size()) {
    tcdrain(fd_);
  }
  close(fd_);
  fd_ = -1;
}

void SerialWriter::thrust_cb(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < thrust_.size(); ++i) {
    thrust_[i] = i < msg->data.size() ? msg->data[i] : 0.0f;
  }
}

void SerialWriter::soft_emg_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    soft_emg_ = msg->data;
    flags_.emergency = soft_emg_ || ground_station_timeout_emg_;
  }
  publish_safety_state();
}

void SerialWriter::red_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  flags_.red = msg->data;
}

void SerialWriter::yellow_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  flags_.yellow = msg->data;
}

void SerialWriter::green_cb(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  flags_.green = msg->data;
}

void SerialWriter::ground_station_heartbeat_cb(const std_msgs::msg::Empty::SharedPtr)
{
  refresh_ground_station_watchdog();
}

void SerialWriter::sbus_command_cb(const geometry_msgs::msg::Twist::SharedPtr)
{
  refresh_ground_station_watchdog();
}

void SerialWriter::refresh_ground_station_watchdog()
{
  std::lock_guard<std::mutex> lock(mutex_);
  ground_station_heartbeat_received_ = true;
  last_ground_station_heartbeat_ = std::chrono::steady_clock::now();
  ground_station_timeout_emg_ = false;
  flags_.emergency = soft_emg_;
}

void SerialWriter::timer_cb()
{
  update_ground_station_watchdog();
  if (fd_ < 0) {return;}
  read_relay_state();
  Packet packet;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    packet = encode_packet(thrust_, flags_, sequence_++);
  }
  const ssize_t result = write(fd_, packet.data(), packet.size());
  if (result != static_cast<ssize_t>(packet.size())) {
    RCLCPP_WARN(get_logger(), "serial write failed or incomplete");
  }
  publish_safety_state();
}

void SerialWriter::read_relay_state()
{
  char buffer[256];
  const ssize_t count = read(fd_, buffer, sizeof(buffer));
  if (count < 0) {
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "serial read failed");
    }
    return;
  }
  for (ssize_t index = 0; index < count; ++index) {
    const auto byte = static_cast<uint8_t>(buffer[index]);
    if (byte == 0x00U || byte == 0x01U) {
      std::lock_guard<std::mutex> lock(mutex_);
      relay_active_ = byte == 0x01U;
    }
  }
}

void SerialWriter::update_ground_station_watchdog()
{
  if (ground_station_heartbeat_timeout_sec_ <= 0.0) {
    return;
  }

  bool timeout = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_ground_station_heartbeat_).count();
    timeout = !ground_station_heartbeat_received_ ||
      elapsed > ground_station_heartbeat_timeout_sec_;
    ground_station_timeout_emg_ = timeout;
    flags_.emergency = soft_emg_ || ground_station_timeout_emg_;
  }
  if (timeout) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "ground-station heartbeat timed out; forcing software emergency stop");
  }
}

void SerialWriter::publish_safety_state()
{
  bool soft_emg = false;
  bool relay_active = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    soft_emg = soft_emg_ || ground_station_timeout_emg_;
    relay_active = relay_active_;
  }
  pub_relay_active_->publish(std_msgs::msg::Bool().set__data(relay_active));
  // GPIO15 follows relay state; it is not an independent physical E-stop input.
  // A commanded soft stop has priority, because it can itself change relay state.
  const auto state = soft_emg ? EmergencyStopState::SOFT_EMG :
    relay_active ? EmergencyStopState::HARD_EMG : EmergencyStopState::RUNNING;
  pub_safety_emergency_->publish(
    std_msgs::msg::UInt8().set__data(
      static_cast<uint8_t>(state)));
}

int SerialWriter::open_serial(const std::string & device, int baud)
{
  int fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    RCLCPP_ERROR(get_logger(), "failed to open serial port %s", device.c_str());
    return -1;
  }
  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    close(fd);
    RCLCPP_ERROR(get_logger(), "tcgetattr failed");
    return -1;
  }
  cfmakeraw(&tty);
  speed_t speed = B115200;
  switch (baud) {
    case 9600: speed = B9600; break;
    case 19200: speed = B19200; break;
    case 38400: speed = B38400; break;
    case 57600: speed = B57600; break;
    case 115200: speed = B115200; break;
    default: speed = B115200; break;
  }
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 5;
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    close(fd);
    RCLCPP_ERROR(get_logger(), "tcsetattr failed");
    return -1;
  }
  int file_flags = fcntl(fd, F_GETFL, 0);
  fcntl(fd, F_SETFL, file_flags | O_NONBLOCK);
  return fd;
}

}  // namespace micon_driver_fd
