# シリアル通信仕様

## PC → ESP

UART は 115200 bps, 8N1 です。PC から ESP32 へ送るデータは
`Docs/PROTOCOL.md` と同じ形式です。

```text
COBS(raw_frame) || 0x00
```

`raw_frame` は 24 byte 固定です。

| Offset | Size | Field | Value |
|---:|---:|---|---|
| 0 | 1 B | version | `0x01` |
| 1 | 1 B | type | `0x01` (`THRUSTER_COMMAND`) |
| 2 | 2 B | sequence | little-endian, 送信ごとに増加 |
| 4 | 1 B | payload_length | `17` |
| 5 | 17 B | payload | 下記 |
| 22 | 2 B | crc | CRC-16/CCITT-FALSE, little-endian |

CRC は version から payload 末尾までの 22 byte を対象にする。

## Payload（17 バイト）

| フィールド | 型 | サイズ | 説明 |
|---|---|---|---|
| th1 | float32 | 4 B | スラスタ 1 の推力（N） |
| th2 | float32 | 4 B | スラスタ 2 の推力（N） |
| th3 | float32 | 4 B | スラスタ 3 の推力（N） |
| th4 | float32 | 4 B | スラスタ 4 の推力（N） |
| setting | uint8 | 1 B | 各種制御フラグ（後述） |

推力からデューティ比への変換には `thrust_duty_profile.hpp` を使用すること。

### setting バイト（各 1 bit）

```
  bit7  bit6  bit5  bit4   bit3     bit2    bit1    bit0
[  0  ][  0  ][  0  ][  0  ][ 緊急停止 ][ LED緑 ][ LED黄 ][ LED赤 ]
```

| ビット | 内容 |
|---|---|
| bit3 | 緊急停止（1 = 停止指令） |
| bit2 | LED 緑（1 = 点灯） |
| bit1 | LED 黄（1 = 点灯） |
| bit0 | LED 赤（1 = 点灯） |

---

## 安全動作

不正 frame、CRC 不一致、非有限 float は出力へ反映しない。

software emergency stop が 1、または有効な指令が 1000 ms 途絶した場合、全スラスターを
中立にして FET を LOW にする。GPIO15 は物理 E-stop の独立した入力ではなく、リレーの
状態を読む信号である。soft emergency stop によってリレーを切った場合もこの信号は変化
し得るため、GPIO15だけから物理 E-stop の作動を判定してはならない。

## ESP → PC

有効な指令フレームを処理するたび、firmware は GPIO15 のリレー状態を表す 1 byte を返す。

```
  bit7  bit6  bit5  bit4  bit3  bit2  bit1      bit0
[  0  ][  0  ][  0  ][  0  ][  0  ][  0  ][  0  ][ 緊急停止リレー動作状態 ]
```

| ビット | 内容 |
|---|---|
| bit0 | リレーの動作状態（1 = 動作中）。物理 E-stop の独立した判定には使えない。 |

---

## ESP32 への書き込み（この環境）

このディレクトリにはスケッチ本体 (`firm1.ino`) と必要なヘッダが同居している。
ここでは搭載ボードを **ESP32 Dev Module**（FQBN: `esp32:esp32:esp32`）としている。
異なるESP32派生ボードの場合は、FQBNを対象ボードのものに置き換えること。

### 1. 接続確認

データ通信できるUSBケーブルでESP32を接続し、ポートを確認する。

```bash
arduino-cli board list
```

出力の `Port` 列に表示された値（例: `/dev/ttyUSB0` または `/dev/ttyACM0`）を以降の
`<PORT>` に指定する。ポートが表示されない場合は、ケーブル、USB-serialドライバ、
およびボードのBOOT/EN操作を確認する。権限エラー時はログインし直すか、利用者を
`dialout` グループに追加してから再ログインする。

### 2. ビルド

この環境では `arduino-cli 1.5.1` と `esp32:esp32 3.3.10` を確認済みである。
未導入の環境では先にESP32 coreを導入する。

```bash
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli compile --fqbn esp32:esp32:esp32 Docs/firm1
```

### 3. 書き込み

スラスター電源を切り、プロペラに触れない安全な状態で実行する。

```bash
arduino-cli upload --fqbn esp32:esp32:esp32 --port <PORT> Docs/firm1
```

`Connecting...` で止まる場合は、ESP32の **BOOT** を押したままコマンドを開始し、
接続が始まったら離す。書き込み後はUSBシリアルを使うROSノードを起動する前に、
他のシリアルモニタを閉じる。

### 4. 動作確認

ESP32は115200 bpsで動作する。有効なスラスタ指令フレームを処理するたびに、
GPIO15の状態を1 byteで返信する。`0x01` はリレーが動作中、`0x00` は非動作を示す。
soft emergency stopでもこの値は変化し得るため、物理E-stopの状態として解釈しない。
