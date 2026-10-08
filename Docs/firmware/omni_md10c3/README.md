# 3輪オムニ MD10C firmware（出力ロック状態）

これは3輪オムニ台車用の専用ESP32 firmwareである。既存の水上機用4 ESC firmware
[`../../firm1/firm1.ino`](../../firm1/firm1.ino)を置き換えない。水上機は従来どおり
message type `0x01`・4個の推力 `[N]` を使用する。本firmwareは type `0x02`・3個の
signed duty ratioだけを受理するため、誤ったprofileのパケットを出力に反映しない。

## 基板照合済みI/O

ユーザー指定の `Njord_2025_Thruster Controle.kicad_pcb` で照合した対応は以下である。

| 輪（ROS順） | 位置角 | 基板ネット | ESP32 PWM GPIO | DIR GPIO |
|---|---:|---|---:|---:|
| 左前 `LF` | +60° | PWM1 | 16 | 25 |
| 後 `REAR` | 180° | PWM2 | 4 | 26 |
| 右前 `RF` | -60° | PWM3 | 18 | 27 |

座標は `+x=前、+y=左、反時計回り正`、車輪中心半径は `0.300 m`、外径は
`127 mm`（車輪半径 `0.0635 m`）である。正の駆動方向は円周接線を仮定しているが、
各MD10Cの極性は未確認である。`omni_md10c3.ino` の `kDirectionInverted` を校正結果に
従い**輪ごとに**設定する。配置やGPIOの確認だけから極性を推測してはならない。

## 通信

UARTは115200 bps、8N1、COBS終端 `0x00`、CRC-16/CCITT-FALSEである。

| フィールド | 値 |
|---|---|
| version | `0x01` |
| type | `0x02` (`OMNI_MD10C3_DUTY`) |
| payload | `float32 duty_lf, duty_rear, duty_rf, uint8 flags` |
| raw frame | 20 byte |
| emergency flag | bit3 |

`micon_driver_fd/config/omni_md10c3.yaml` の `command_profile:=md10c3_duty` がこれを送る。
payloadは校正済みの物理速度・回転数・推力ではなく、符号付き duty 比である。

## 安全動作

- 起動時、フレーム不正、CRC不一致、NaN/InfではPWMを更新しない。
- 初期化・soft emergency stop・250 msの有効通信途絶では、全PWMを0、全DIRをLOWにする。
- 有限値もfirmware内で `abs(duty) <= 0.50` に再clampする。host側にも同じ上限がある。
- 正負が切り替わるchは、PWMを0にして2 ms待ってからDIRを切り替える。hostの疑似TTY
  テストでも正負符号がそのまま専用packetへ保存されることを確認する。
- `kMotorOutputEnabled` は既定で `false`。そのままコンパイルしても出力は必ず0である。
- この基板のMD10C profileには検証済みの独立した物理E-stop入力を定義していない。実機の
  電源遮断・非常停止系を別途確認し、出力ロック解除前に試験すること。

## 校正と有効化（実機ではまだ実施しない）

1. 台車を持ち上げるか、車輪を完全に浮かせる。物理E-stop、電源遮断、監視者を準備する。
2. `kMotorOutputEnabled` をまだ `false` のまま、`arduino-cli compile --fqbn esp32:esp32:esp32 Docs/firmware/omni_md10c3` を行う。uploadは電源を切り、明示承認後だけ実施する。
3. wheel 1輪ずつ、`0.05`から開始して `0.05`刻みで最大`0.15`まで、各1秒以下で正負を確認する。
   期待する接線方向と逆なら、その輪だけ `kDirectionInverted` と YAMLの `reverse` のどちらを
   正の定義にするかを一箇所へ決め、二重反転にしない。
4. 3輪とも確認後、host YAMLの `safety.actuator_configuration_confirmed:=true` と
   firmwareの `kMotorOutputEnabled=true` を同じ試験記録で解除する。最初の地上試験は
   duty 0.05以下、1方向ずつ、短時間に限定する。
5. 速度・回頭の校正は外部自己位置（モーションキャプチャ、既知距離のタイム計測、または
   LiDAR/GNSSの妥当性確認済みodometry）で行う。エンコーダ無しのため、`cmd_vel`を物理速度と
   解釈せず、方向ごとの不感帯・正負非対称・電圧低下を別表に記録する。

`abs(duty) <= 0.50` は安全保証ではない。衝突、転倒、電流、配線、床面に応じて上の開始値より
さらに下げ、異常時はただちに物理E-stopと電源遮断を使う。
