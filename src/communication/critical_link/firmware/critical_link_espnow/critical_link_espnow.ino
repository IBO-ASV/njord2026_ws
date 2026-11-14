#include <Arduino.h>
#include <WiFi.h>
#include <esp_arduino_version.h>
#include <esp_now.h>
#include <esp_wifi.h>

#if __has_include("critical_link_config.h")
#include "critical_link_config.h"
#else
#error "Copy critical_link_config.example.h to critical_link_config.h and configure role, peer MAC, channel, PMK and LMK"
#endif

namespace
{

constexpr uint8_t kFrameMagic[4] = {'N', 'C', 'L', '1'};
constexpr uint8_t kAckMagic[4] = {'N', 'C', 'A', 'K'};
constexpr uint8_t kProtocolVersion = 1;
constexpr size_t kHeaderSize = 30;
constexpr size_t kCrcSize = 4;
constexpr size_t kMaxPayloadSize = 200;
constexpr size_t kMaxFrameSize = kHeaderSize + kMaxPayloadSize + kCrcSize;
constexpr size_t kAckSize = 24;
constexpr size_t kSerialBufferSize = 512;

struct RadioPacket
{
  uint16_t size;
  uint8_t data[kMaxFrameSize];
};

struct PendingPacket
{
  bool active{false};
  uint8_t stream{0};
  uint8_t attempts{0};
  uint16_t size{0};
  uint64_t session{0};
  uint32_t sequence{0};
  uint32_t last_send_ms{0};
  uint8_t data[kMaxFrameSize]{};
};

QueueHandle_t receive_queue = nullptr;
uint8_t serial_buffer[kSerialBufferSize]{};
size_t serial_size = 0;
PendingPacket pending[4];

uint16_t read_u16(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0]) |
         static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8U);
}

uint32_t read_u32(const uint8_t * data)
{
  uint32_t value = 0;
  for (uint8_t shift = 0; shift < 32; shift += 8) {
    value |= static_cast<uint32_t>(*data++) << shift;
  }
  return value;
}

uint64_t read_u64(const uint8_t * data)
{
  uint64_t value = 0;
  for (uint8_t shift = 0; shift < 64; shift += 8) {
    value |= static_cast<uint64_t>(*data++) << shift;
  }
  return value;
}

void write_u32(uint8_t * data, uint32_t value)
{
  for (uint8_t shift = 0; shift < 32; shift += 8) {
    *data++ = static_cast<uint8_t>(value >> shift);
  }
}

void write_u64(uint8_t * data, uint64_t value)
{
  for (uint8_t shift = 0; shift < 64; shift += 8) {
    *data++ = static_cast<uint8_t>(value >> shift);
  }
}

uint32_t crc32_ieee(const uint8_t * data, size_t size)
{
  uint32_t crc = 0xFFFFFFFFU;
  for (size_t index = 0; index < size; ++index) {
    crc ^= data[index];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      const uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }
  return ~crc;
}

bool validate_frame(const uint8_t * data, size_t size)
{
  if (size < kHeaderSize + kCrcSize || size > kMaxFrameSize ||
      memcmp(data, kFrameMagic, sizeof(kFrameMagic)) != 0 ||
      data[4] != kProtocolVersion || data[5] < 1 || data[5] > 3) {
    return false;
  }
  const size_t payload_size = read_u16(data + 28);
  return payload_size <= kMaxPayloadSize &&
         size == kHeaderSize + payload_size + kCrcSize &&
         read_u32(data + size - kCrcSize) == crc32_ieee(data, size - kCrcSize);
}

bool validate_ack(const uint8_t * data, size_t size)
{
  return size == kAckSize && memcmp(data, kAckMagic, sizeof(kAckMagic)) == 0 &&
         data[4] == kProtocolVersion && data[5] >= 1 && data[5] <= 3 &&
         read_u32(data + kAckSize - kCrcSize) == crc32_ieee(data, kAckSize - kCrcSize);
}

void send_ack(const RadioPacket & packet)
{
  uint8_t ack[kAckSize]{};
  memcpy(ack, kAckMagic, sizeof(kAckMagic));
  ack[4] = kProtocolVersion;
  ack[5] = packet.data[5];
  write_u64(ack + 8, read_u64(packet.data + 8));
  write_u32(ack + 16, read_u32(packet.data + 16));
  write_u32(ack + 20, crc32_ieee(ack, 20));
  esp_now_send(CRITICAL_LINK_PEER_MAC, ack, sizeof(ack));
}

void queue_pending(const uint8_t * data, size_t size)
{
  const uint8_t stream = data[5];
  if (stream >= 4) {
    return;
  }
  auto & slot = pending[stream];
  slot.active = true;
  slot.stream = stream;
  slot.attempts = 1;
  slot.size = static_cast<uint16_t>(size);
  slot.session = read_u64(data + 8);
  slot.sequence = read_u32(data + 16);
  slot.last_send_ms = millis();
  memcpy(slot.data, data, size);
  esp_now_send(CRITICAL_LINK_PEER_MAC, slot.data, slot.size);
}

void accept_ack(const uint8_t * data)
{
  const uint8_t stream = data[5];
  if (stream >= 4) {
    return;
  }
  auto & slot = pending[stream];
  if (slot.active && slot.session == read_u64(data + 8) &&
      slot.sequence == read_u32(data + 16)) {
    slot.active = false;
  }
}

void handle_radio_receive(const uint8_t * data, int length)
{
#if CRITICAL_LINK_ROLE_GROUND
  if (length == static_cast<int>(kAckSize) && validate_ack(data, kAckSize)) {
    RadioPacket packet{};
    packet.size = static_cast<uint16_t>(length);
    memcpy(packet.data, data, packet.size);
    xQueueSend(receive_queue, &packet, 0);
  }
#else
  if (length <= 0 || length > static_cast<int>(kMaxFrameSize) ||
      !validate_frame(data, static_cast<size_t>(length))) {
    return;
  }
  RadioPacket packet{};
  packet.size = static_cast<uint16_t>(length);
  memcpy(packet.data, data, packet.size);
  xQueueSend(receive_queue, &packet, 0);
#endif
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void on_data_received(const esp_now_recv_info_t *, const uint8_t * data, int length)
#else
void on_data_received(const uint8_t *, const uint8_t * data, int length)
#endif
{
  handle_radio_receive(data, length);
}

void process_serial()
{
  while (Serial.available() > 0) {
    const int value = Serial.read();
    if (value < 0) {
      break;
    }
    if (serial_size == kSerialBufferSize) {
      serial_size = 0;
    }
    serial_buffer[serial_size++] = static_cast<uint8_t>(value);
  }

  while (serial_size >= sizeof(kFrameMagic)) {
    size_t magic_offset = 0;
    while (magic_offset + sizeof(kFrameMagic) <= serial_size &&
           memcmp(serial_buffer + magic_offset, kFrameMagic, sizeof(kFrameMagic)) != 0) {
      ++magic_offset;
    }
    if (magic_offset > 0) {
      memmove(serial_buffer, serial_buffer + magic_offset, serial_size - magic_offset);
      serial_size -= magic_offset;
    }
    if (serial_size < kHeaderSize) {
      return;
    }
    const size_t payload_size = read_u16(serial_buffer + 28);
    if (payload_size > kMaxPayloadSize) {
      memmove(serial_buffer, serial_buffer + 1, --serial_size);
      continue;
    }
    const size_t frame_size = kHeaderSize + payload_size + kCrcSize;
    if (serial_size < frame_size) {
      return;
    }
    if (validate_frame(serial_buffer, frame_size)) {
#if CRITICAL_LINK_ROLE_GROUND
      queue_pending(serial_buffer, frame_size);
#endif
      memmove(serial_buffer, serial_buffer + frame_size, serial_size - frame_size);
      serial_size -= frame_size;
    } else {
      memmove(serial_buffer, serial_buffer + 1, --serial_size);
    }
  }
}

void retry_pending()
{
#if CRITICAL_LINK_ROLE_GROUND
  const uint32_t now = millis();
  for (uint8_t stream = 1; stream <= 3; ++stream) {
    auto & slot = pending[stream];
    if (!slot.active || now - slot.last_send_ms < CRITICAL_LINK_RETRY_INTERVAL_MS) {
      continue;
    }
    if (slot.attempts >= CRITICAL_LINK_MAX_ATTEMPTS) {
      slot.active = false;
      continue;
    }
    ++slot.attempts;
    slot.last_send_ms = now;
    esp_now_send(CRITICAL_LINK_PEER_MAC, slot.data, slot.size);
  }
#endif
}

void flush_received_to_serial()
{
  RadioPacket packet{};
  while (xQueueReceive(receive_queue, &packet, 0) == pdTRUE) {
#if CRITICAL_LINK_ROLE_GROUND
    if (packet.size == kAckSize && validate_ack(packet.data, packet.size)) {
      accept_ack(packet.data);
    }
#else
    Serial.write(packet.data, packet.size);
    Serial.flush();
    send_ack(packet);
#endif
  }
}

void stop_with_error(const char * message, esp_err_t error)
{
  Serial.printf("%s: %s\n", message, esp_err_to_name(error));
  while (true) {
    delay(1000);
  }
}

}  // namespace

void setup()
{
  Serial.begin(CRITICAL_LINK_SERIAL_BAUD);
  delay(500);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  esp_err_t result = esp_wifi_set_channel(
    CRITICAL_LINK_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (result != ESP_OK) {
    stop_with_error("esp_wifi_set_channel failed", result);
  }
  result = esp_now_init();
  if (result != ESP_OK) {
    stop_with_error("esp_now_init failed", result);
  }
  result = esp_now_set_pmk(CRITICAL_LINK_PMK);
  if (result != ESP_OK) {
    stop_with_error("esp_now_set_pmk failed", result);
  }

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, CRITICAL_LINK_PEER_MAC, sizeof(peer.peer_addr));
  memcpy(peer.lmk, CRITICAL_LINK_LMK, sizeof(peer.lmk));
  peer.channel = CRITICAL_LINK_ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = true;
  result = esp_now_add_peer(&peer);
  if (result != ESP_OK && result != ESP_ERR_ESPNOW_EXIST) {
    stop_with_error("esp_now_add_peer failed", result);
  }
  receive_queue = xQueueCreate(8, sizeof(RadioPacket));
  if (receive_queue == nullptr) {
    stop_with_error("xQueueCreate failed", ESP_ERR_NO_MEM);
  }
  esp_now_register_recv_cb(on_data_received);

  Serial.printf(
    "critical-link role=%s mac=%s channel=%u max_frame=%u\n",
    CRITICAL_LINK_ROLE_GROUND ? "ground" : "vessel",
    WiFi.macAddress().c_str(), CRITICAL_LINK_ESPNOW_CHANNEL,
    static_cast<unsigned int>(kMaxFrameSize));
}

void loop()
{
  process_serial();
  retry_pending();
  flush_received_to_serial();
  delay(1);
}
