# ESP32 Thruster Firmware

ESP32で4基のスラスター、非常停止用FET、状態表示LEDを制御するArduinoファームウェアです。PCからUARTで単方向のスラスター指令を受信します。

## 主な機能

- 4基のスラスターを50 Hz・12 bit PWMで制御
- `float32`の推力値（N）を測定データからPWMパルス幅へ線形補間
- COBSフレーミングとCRC-16による通信エラー検出
- GPIO2 に接続した物理非常停止リレーの監視と状態通知
- software emergency stopによる停止
- 有効な制御コマンドが1000 ms途絶した場合のフェイルセーフ停止

## ファイル構成

| ファイル | 内容 |
|---|---|
| `firm1/firm1.ino` | GPIO、PWM、安全状態、UART受信およびコマンド処理 |
| `firm1/serial_protocol.hpp` | COBS、CRC、プロトコル定数およびバイト変換 |
| `firm1/thrust_duty_profile.hpp` | 推力からPWMパルス幅への変換 |
| `PROTOCOL.md` | UART通信プロトコルの正式仕様 |

## ピン配置

| 用途 | GPIO |
|---|---:|
| Servo 1 | 16 |
| Servo 2 | 4 |
| Servo 3 | 18 |
| Servo 4 | 17 |
| FET switch | 14 |
| Green LED | 25 |
| Yellow LED | 26 |
| Red LED | 27 |
| Emergency-stop relay input | 2 |

論理スラスター番号とServo出力の対応は次のとおりです。

| 推力指令 | 出力先 |
|---|---|
| `thrust_1` | Servo 4（GPIO 17） |
| `thrust_2` | Servo 1（GPIO 16） |
| `thrust_3` | Servo 3（GPIO 18） |
| `thrust_4` | Servo 2（GPIO 4） |

## シリアル通信

通信条件は115200 bps、8 data bits、no parity、1 stop bitです。

送信フレームは次の形式です。

```text
COBS(Version | Type | Sequence | PayloadLength | Payload | CRC-16) | 0x00
```

詳細なフィールド、CRCパラメーターおよび異常時動作は
[`PROTOCOL.md`](PROTOCOL.md)を参照してください。

## 安全動作

- 起動時は全スラスターを中立（1500 us）にし、FETをLOWにします。
- ARM/DISARM状態はなく、有効な通常指令を受信するとFETをHIGHにします。
- software emergency stop、GPIO2 の物理非常停止、または通信非常停止時は、全スラスターを中立にしてFETをLOWにします。
- 非常停止はラッチせず、解除後の次の有効な指令から出力を再開します。
- 有効な推力コマンドが1000 ms途絶すると、全スラスターを中立にしてFETをLOWにします。
- CRC、長さ、Version、TypeまたはPayload検査に失敗したデータは出力へ反映せず、通信ウォッチドッグも更新しません。

> [!IMPORTANT]
> 実機で推力を出す前に、GPIO配線、スラスターの回転方向、FETの論理、非常停止リレーの論理を必ず確認してください。

## 開発環境

ESP32向けArduino環境を前提とします。使用するESP32 Arduino Coreは、ピン番号を指定する`ledcAttach(pin, frequency, resolution)`および`ledcWrite(pin, duty)` APIに対応している必要があります。
