#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "../common/telemetry_protocol.h"

#define ESPNOW_CHANNEL 1

namespace {

constexpr uint32_t kOutputIntervalMs = 1000;
constexpr uint32_t kStaleAfterMs = 2000;

struct CellState {
  bms::CellTelemetry packet{};
  uint32_t received_ms = 0;
  uint8_t source_mac[6]{};
  bool present = false;
};

CellState cells[bms::kCellCount];
portMUX_TYPE cells_lock = portMUX_INITIALIZER_UNLOCKED;
uint32_t next_output_ms = 0;

void receivePacket(const uint8_t *source_mac, const uint8_t *data, int length) {
  if (length != static_cast<int>(sizeof(bms::CellTelemetry))) return;

  bms::CellTelemetry packet{};
  memcpy(&packet, data, sizeof(packet));
  if (!bms::isValid(packet)) return;

  CellState &cell = cells[packet.cell_id - 1];
  portENTER_CRITICAL(&cells_lock);
  cell.packet = packet;
  cell.received_ms = millis();
  memcpy(cell.source_mac, source_mac, sizeof(cell.source_mac));
  cell.present = true;
  portEXIT_CRITICAL(&cells_lock);
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *data,
                    int length) {
  receivePacket(info->src_addr, data, length);
}
#else
void onDataReceived(const uint8_t *mac, const uint8_t *data, int length) {
  receivePacket(mac, data, length);
}
#endif

void stopWithError(const char *message, esp_err_t error) {
  Serial.printf("FATAL: %s (error=%d)\n", message, static_cast<int>(error));
  while (true) delay(1000);
}

void setupRadio() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_err_t result = esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (result != ESP_OK) stopWithError("cannot set Wi-Fi channel", result);
  result = esp_now_init();
  if (result != ESP_OK) stopWithError("ESP-NOW init failed", result);
  esp_now_register_recv_cb(onDataReceived);
}

void printValueOrNan(bool valid, float value) {
  if (valid) Serial.print(value, 4);
  else Serial.print("nan");
}

void outputCsv() {
  CellState snapshot[bms::kCellCount];
  portENTER_CRITICAL(&cells_lock);
  memcpy(snapshot, cells, sizeof(snapshot));
  portEXIT_CRITICAL(&cells_lock);

  const uint32_t now = millis();
  bool valid[bms::kCellCount]{};
  bool fresh[bms::kCellCount]{};
  bool all_valid = true;
  bool adc_fault = false;
  float total = 0.0F;

  Serial.print(now);
  for (uint8_t i = 0; i < bms::kCellCount; ++i) {
    fresh[i] = snapshot[i].present &&
               (static_cast<uint32_t>(now - snapshot[i].received_ms) <=
                kStaleAfterMs);
    const bool saturated = fresh[i] &&
                           (snapshot[i].packet.flags & bms::kAdcSaturated);
    adc_fault |= saturated;
    valid[i] = fresh[i] && !saturated;
    all_valid &= valid[i];
    if (valid[i]) total += snapshot[i].packet.cell_voltage_v;
    Serial.print(',');
    printValueOrNan(valid[i], snapshot[i].packet.cell_voltage_v);
  }

  Serial.print(',');
  printValueOrNan(all_valid, total);

  const bool dht20_valid =
      fresh[0] && (snapshot[0].packet.flags & bms::kTemperatureValid) &&
      isfinite(snapshot[0].packet.temperature_c) &&
      isfinite(snapshot[0].packet.humidity_percent);
  Serial.print(',');
  printValueOrNan(dht20_valid, snapshot[0].packet.temperature_c);
  Serial.print(',');
  printValueOrNan(dht20_valid, snapshot[0].packet.humidity_percent);

  for (uint8_t i = 0; i < bms::kCellCount; ++i) {
    Serial.print(',');
    if (snapshot[i].present) Serial.print(now - snapshot[i].received_ms);
    else Serial.print("nan");
  }
  Serial.print(',');
  if (adc_fault) Serial.println("ADC_FAULT");
  else if (!all_valid) Serial.println("STALE");
  else if (!dht20_valid) Serial.println("DHT20_FAULT");
  else Serial.println("OK");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  setupRadio();
  Serial.printf("Master ready, MAC=%s, channel=%u\n", WiFi.macAddress().c_str(),
                ESPNOW_CHANNEL);
  Serial.println(
      "ms,cell1_V,cell2_V,cell3_V,cell4_V,total_V,cell1_temp_C,cell1_humidity_pct,age1_ms,age2_ms,age3_ms,age4_ms,status");
}

void loop() {
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - next_output_ms) < 0) return;
  next_output_ms = now + kOutputIntervalMs;
  outputCsv();
}
