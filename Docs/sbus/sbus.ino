#include <Arduino.h>
#include <sbus.h>

namespace {

// XIAO ESP32-C6 の基板上で「RX」と印刷されている端子 (D7 / GPIO17)
constexpr int8_t kSbusRxPin = D7;
constexpr int8_t kUnusedTxPin = -1;

// true: 通常の反転 SBUS (100000 baud)
// 最後の引数を false にすると、非反転信号用になる。
bfs::SbusRx sbus_rx(&Serial1, kSbusRxPin, kUnusedTxPin, true);

uint32_t frame_count = 0;

void PrintHeader() {
  Serial.print("millis,frame");
  for (uint8_t channel = 1; channel <= 16; ++channel) {
    Serial.print(",ch");
    Serial.print(channel);
  }
  Serial.println(",ch17,ch18,lost_frame,failsafe");
}

void PrintFrame(const bfs::SbusData &data) {
  Serial.print(millis());
  Serial.print(',');
  Serial.print(++frame_count);

  for (int8_t channel = 0; channel < bfs::SbusData::NUM_CH; ++channel) {
    Serial.print(',');
    Serial.print(data.ch[channel]);
  }

  Serial.print(',');
  Serial.print(data.ch17 ? 1 : 0);
  Serial.print(',');
  Serial.print(data.ch18 ? 1 : 0);
  Serial.print(',');
  Serial.print(data.lost_frame ? 1 : 0);
  Serial.print(',');
  Serial.println(data.failsafe ? 1 : 0);
}

}  // namespace

void setup() {
  // SBUS が 100 Hz の場合でも全項目を余裕を持って出力できる速度。
  Serial.begin(230400);

  // USB シリアル接続を少しだけ待つ。未接続でも2秒後には受信を開始する。
  const uint32_t wait_started_ms = millis();
  while (!Serial && millis() - wait_started_ms < 2000) {
    delay(10);
  }

  PrintHeader();
  sbus_rx.Begin();
}

void loop() {
  if (sbus_rx.Read()) {
    PrintFrame(sbus_rx.data());
  }
}
