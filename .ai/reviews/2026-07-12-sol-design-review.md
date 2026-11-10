# Codex Sol 設計レビュー — sim↔real ブリッジ

日時: 2026-07-12 / レビュアー: Codex Sol (gpt-5.5, read-only) / 対象: 実装前設計

## 結論(採用)
1. **BMS 受信 = 単一 serial 所有ノードに TX/RX 集約 (c)**。`micon_driver_fd` を双方向化し、
   1プロセスだけが tty を open。別ノードで同一 tty を open する設計は禁止(所有権曖昧・stream 競合)。
   RX は read thread or nonblocking timer read、TX 50ms timer は止めない。reconnect/backoff も一箇所。
2. **RX フレーミング必須**: 開始バイト+type+length+seq+payload+CRC。生バイト混在は同期ずれ/部分read/ESP32 reset で誤decode。
3. **Newton 不変条件**: `/thruster_command` は常に Float32MultiArray[4] の Newton。normalized は thruster_driver 内部のみ。
   serial_writer は再スケールせず memcpy。README の normalized/Int16 記述は誤り→訂正。
4. **CAN/mros 除去**: thruster_driver の transport_mode 廃止 or sim 固定、CAN/mros publisher・can_msgs 依存・UInt16 変換削除。
   非sim 指定は**起動エラー**(実機安全)。thruster_driver pkg 自体はテストが allocation.hpp を参照するため残す。
5. **実機 bringup**: `robot/launch/real_bringup.launch.py` 新設、既存個別 launch を include、
   `enable_mid360/zed2i/back_cam/um982/imu/localization/thruster/nav2` 引数化、全実機系 use_sim_time=false。
6. **安全**: 起動時 0N / cmd timeout 0N / feedback timeout 0N / serial disconnect 0N を acceptance に。
   注意: `serial_writer` の `/emg` callback は値を無視し常に emergency=false(SW E-stop 未接続)。
   `manual_control.launch.py` は stop_on_feedback_timeout=false → 実機 bringup では true 固定。

## 指摘(落とし穴)
- 同一 tty を bms node と writer が別々に open する設計は避ける。
- TX 生 byte のまま RX を足すと同期ずれから復帰不能になりやすい。
- `pub_current_force_` は実際は normalized command を publish、名前が紛らわしい。
- `force_per_duty` 名は内部 normalized→N 変換で、外部に duty 単位を誤想起させる。
- kinematics 削除後、thruster_config.yaml / 各 README / task1_sim.launch.py 等に参照残の恐れ。

## テスト観点(acceptance 追加)
- TX encode: 4 float Newton LE + flag bit、起動直後 0N。
- RX parser: 分割read/連結frame/先頭ゴミ/CRC NG/length NG/再同期。
- BMS publish: 4セルV固定長、NaN/範囲外の扱い。
- timeout: cmd_vel停止/odom停止/write失敗 → 0N。
- 単位: /thruster_command = command*force_per_duty、bridge は再スケールしない。
- 削除確認: colcon build + `rg can_msgs|kinematics|mros_usb|transport_mode` 残存なし。
- bringup dry-run: enable フラグ別に node/topic 構成を ros2 topic hz/echo で確認。

## 司令塔(メインClaude)の scope 裁定(Sol 提案への補足)
- **ESP32 ファーム変更は本リポジトリ外・本タスクのスコープ外**(完了ライン=コード/build/test/dryrun)。
  従い:
  - **TX は現行 raw v1(`4×float32 LE + flag byte`)を維持**(既存ファーム契約を黙って壊さない)。プロトコル名として明示ドキュメント化。
  - **RX(BMS)は framed+CRC をホスト側契約として実装**し、loopback ユニットテストで検証。
    実 BMS が流れるのはファーム対応後(HW統合工程)= dryrun/test では parser を単体検証。
- **bms pkg は残す**: 単一 tty 所有は micon_driver_fd。bms pkg は所有ノードが出す復号済み topic を購読し
  4セル標準化 topic (`bms/cell_voltages`) + しきい値/diagnostics を担う(tty は open しない)。
- **serial_writer 実行体名は維持**(manual_control.launch.py が参照)。双方向化しても executable/node 名は据え置き。
