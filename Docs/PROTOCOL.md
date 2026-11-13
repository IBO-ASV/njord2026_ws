# ESP32スラスター通信プロトコル仕様書

## 概要

PCからESP32へスラスター指令を一方向に送信する単純なプロトコルです。
ARM、DISARM、STATUS応答は使用しません。

UARTは115200 bps、8N1、フロー制御なしです。複数バイト値は
little-endian、浮動小数点はIEEE 754 binary32とします。

## フレーミング

送信データは次の形式です。

```text
COBS(raw_frame) || 0x00
```

`0x00`をフレーム終端とし、破損フレームを受信した場合は次の終端から
受信を再開します。raw frameの最大長は64 byteです。

## Raw frame

| Offset | Size | Type | Field | Value |
|---:|---:|---|---|---|
| 0 | 1 | `uint8` | version | `0x01` |
| 1 | 1 | `uint8` | type | `0x01` (`THRUSTER_COMMAND`) |
| 2 | 2 | `uint16` | sequence | 送信ごとに1増加 |
| 4 | 1 | `uint8` | payload_length | `17` |
| 5 | 17 | byte array | payload | 下記参照 |
| 22 | 2 | `uint16` | crc | CRC-16/CCITT-FALSE |

raw frameは常に24 byteです。

## Payload

| Payload offset | Size | Type | Field |
|---:|---:|---|---|
| 0 | 4 | `float32` | thrust_1 [N] |
| 4 | 4 | `float32` | thrust_2 [N] |
| 8 | 4 | `float32` | thrust_3 [N] |
| 12 | 4 | `float32` | thrust_4 [N] |
| 16 | 1 | `uint8` | control_flags |

`control_flags`は次の割り当てです。

| Bit | Mask | Meaning |
|---:|---:|---|
| 0 | `0x01` | Red LED |
| 1 | `0x02` | Yellow LED |
| 2 | `0x04` | Green LED |
| 3 | `0x08` | Software emergency stop |
| 4..7 | `0xF0` | Reserved |

software emergency stopが1、またはGPIO2の物理E-stop入力がLOWの場合、ESP32は
全スラスターを中立にしてFETをLOWにします。それ以外の有効な指令ではFETをHIGHに
して4スラスターの指令を反映します。非常停止状態はラッチせず、
解除後の次の有効な指令から出力を再開します。

有効な指令を1000 ms受信しなかった場合も、ESP32は全スラスターを中立にして
FETをLOWにします。

## PC側の推力指令watchdog

`micon_driver_fd/serial_writer` は `/thruster_command` の受信をsteady clockで監視します。
起動後に有効指令が来ていない場合、最後の有効指令から `command_timeout_sec`（既定0.5秒）
を超えた場合、配列が4要素でない場合、NaN/Infを含む場合は、4基ゼロ＋software emergency stop＋赤を送ります。
LED・地上局・SBUS通信だけではこの期限を更新しません。新しい有効な推力指令で復帰しますが、
独立したsoftware emergency stopや地上局watchdogは維持します。
`/micon/thruster_command_status` と `/micon/thruster_command_fresh` に受信状態を公開します。

ESP32のUART無受信1秒停止とは別の監視です。UARTのフレーム形式は変更していません。

## CRC

CRCはversionからpayload末尾までの22 byteを対象とします。

| Parameter | Value |
|---|---:|
| Width | 16 |
| Poly | `0x1021` |
| Init | `0xFFFF` |
| RefIn / RefOut | false / false |
| XorOut | `0x0000` |
| Check (`123456789`) | `0x29B1` |

CRC値はlittle-endianで格納します。長さ、version、type、CRC、有限なfloat値の
いずれかが不正なフレームは出力へ反映しません。

## Relay status response

有効な指令フレームを処理するたび、ESP32はUARTへ1 byteのリレー状態を返します。
bit0が`1`ならGPIO2はLOW（非常停止リレー動作中）、`0`なら解除状態です。bit1..7は
常に`0`です。
