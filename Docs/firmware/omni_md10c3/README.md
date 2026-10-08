# 3輪オムニ MD10C firmware（既定ロック・校正専用モード付き）

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

- 起動時、PWM初期化失敗、フレーム不正、CRC不一致、NaN/Infでは**全chを直ちに
  PWM=0、DIR=LOW**にする。途中まで届いたフレームが50 msを超えた場合も破棄し、次の
  delimiterまで後半だけを受理しない。
- 初期化・soft emergency stop・250 msの有効通信途絶では、全PWMを0、全DIRをLOWにする。
- 有限値もfirmware内で `abs(duty) <= 0.50` に再clampする。host側にも同じ上限がある。
- 正負が切り替わるchは、PWMを0にして2 ms待ってからDIRを切り替える。hostの疑似TTY
  テストでも正負符号がそのまま専用packetへ保存されることを確認する。
- `kMotorOutputEnabled` と `kCalibrationOutputEnabled` は既定で `false`。そのまま
  コンパイルしても出力は必ず0である。両方を同時に `true` にするとコンパイルエラーに
  なる。校正モードは一輪のみ、`abs(duty) <= 0.15`、連続1秒で停止ラッチとする。
- PWMはMD10Cが許容する最大20 kHzのまま、ESP32 LEDCで成立する11 bitを使う。各
  `ledcAttach` / 初期化時のゼロ書込みを確認し、失敗時はPWMをdetachして全GPIOをLOWの
  ままラッチする。
- この基板のMD10C profileには検証済みの独立した物理E-stop入力を定義していない。実機の
  電源遮断・非常停止系を別途確認し、出力ロック解除前に試験すること。

## 校正と有効化（実機ではまだ実施しない）

1. 台車を持ち上げるか、車輪を完全に浮かせる。物理E-stop、電源遮断、監視者を準備する。
2. まず両方の出力flagを `false` のままcompileして、出力ロック状態であることを確認する。
   uploadは電源を切り、明示承認後だけ実施する。
3. **極性確認だけ**を許可するとき、`kMotorOutputEnabled=false` を保ち、
   `kCalibrationOutputEnabled=true` だけにしてbuild/uploadする。通常profileではなく、下の
   3個を必ず組にして選び、`/omni_calibration_duty_int16` に一輪だけのInt16配列を送る。

   ```text
   thruster_config_file:=<thruster_driver>/config/omni_md10c3_calibration.yaml
   thruster_robot_description_file:=<robot>/urdf/omni_3wheel.urdf
   thruster_serial_config_file:=<micon_driver_fd>/config/omni_md10c3_calibration.yaml
   ```

   例: `[50, 0, 0]` はLFだけに +0.05 duty を要求する。hostは0.15、firmwareは
   一輪・0.15・連続1秒で二重に制限し、ゼロまたはemergency frameを受けるまで次のpulseを
   拒否する。実機への送信は本手順書だけでは許可されない。
4. 期待する接線方向と逆なら、その輪だけfirmwareの `kDirectionInverted` を変更する。
   `duty_array` はYAMLの `reverse` を通らないため、校正時に通常profileの `reverse` を
   併用して補正してはならない。通常profileの `reverse` は `false` のまま維持する。
5. 3輪の極性・物理停止・配線記録をレビュー後、校正flagを再び `false` にし、通常運用を
   許可する場合だけ `kMotorOutputEnabled=true` と通常YAMLの
   `safety.actuator_configuration_confirmed:=true` を同じ試験記録で解除する。最初の地上試験は
   duty 0.05以下、1方向ずつ、短時間に限定する。
6. 速度・回頭の校正は外部自己位置（モーションキャプチャ、既知距離のタイム計測、または
   LiDAR/GNSSの妥当性確認済みodometry）で行う。エンコーダ無しのため、`cmd_vel`を物理速度と
   解釈せず、方向ごとの不感帯・正負非対称・電圧低下を別表に記録する。

`abs(duty) <= 0.50` は安全保証ではない。衝突、転倒、電流、配線、床面に応じて上の開始値より
さらに下げ、異常時はただちに物理E-stopと電源遮断を使う。
