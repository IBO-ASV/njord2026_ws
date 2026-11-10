# XIAO ESP32-C6 SBUS reader

Seeed Studio XIAO ESP32-C6 の RX 端子でラジコン受信機の SBUS を読み、受信した全項目を USB シリアルへ CSV 形式で出力するテスト用スケッチです。

## 配線

| SBUS受信機 | XIAO ESP32-C6 |
|---|---|
| SBUS信号 | RX (D7 / GPIO17) |
| GND | GND |

受信機とXIAOのGNDは必ず共通にしてください。ESP32-C6のGPIOは3.3 V系です。受信機のSBUS出力電圧が3.3 Vを超える場合は、適切なレベル変換を入れてください。

一般的な反転SBUSを想定しています。bolderflight/sbus が ESP32 のUART反転機能を使用するため、外付けインバータは不要です。すでに外部回路で非反転にしている信号なら、`sbus_rx` のコンストラクタ末尾を `false` に変更します。

## Arduino IDE

1. ボードマネージャーで ESP32 ボードパッケージを導入します。
2. ボードを `XIAO_ESP32C6` に設定します。
3. [bolderflight/sbus](https://github.com/bolderflight/sbus) をダウンロードし、Arduinoライブラリとして追加します。
4. `sbus.ino` を開いて書き込みます。
5. シリアルモニタを `230400 baud` に設定します。

出力例:

```text
millis,frame,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8,ch9,ch10,ch11,ch12,ch13,ch14,ch15,ch16,ch17,ch18,lost_frame,failsafe
2031,1,992,992,172,992,992,992,992,992,992,992,992,992,992,992,992,992,0,0,0,0
```

- `ch1`〜`ch16`: 生の11 bitチャンネル値（一般的なFrSkyではおよそ172〜1811）
- `ch17`, `ch18`: デジタルチャンネル（0または1）
- `lost_frame`: フレーム欠落フラグ
- `failsafe`: フェイルセーフ作動フラグ

何も表示されない場合は、GND共通、受信機側のSBUS出力設定、信号電圧、反転・非反転の順に確認してください。
