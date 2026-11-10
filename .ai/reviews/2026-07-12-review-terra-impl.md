# 実装レビュー — Terra 実装 (sim↔real ブリッジ)

日時: 2026-07-12 / レビュアー: メインClaude(Opus, 実装者=Codex Terra gpt-5.5 とは別モデル)
対象: Terra 実装差分(micon 一式 + real_bringup.launch.py + sim/config 掃除)

## 検証結果(独立実行)
- `colcon build --packages-select thruster_driver micon_driver_fd bms`(ROS2 **Jazzy**)= 成功。
- `colcon test` 同3pkg = **44 tests, 0 errors, 0 failures, 7 skipped**(Terra 報告と一致)。
- `rg 'can_msgs|kinematics|mros_usb|transport_mode' src` = 残存なし。
- real_bringup 参照 launch/引数を実在確認: zed2i(mode=sdk 宣言済), livox `launch_ROS2/msg_MID360_launch.py`,
  wit `wit_node/launch/wit.launch.py`, thruster `stop_on_feedback_timeout` 宣言済, um982, back_cam(video_device)。全て実在。

## 良い点(設計準拠を確認)
- 単一 tty 所有(micon_driver_fd のみ open)。TX raw v1 維持。executable 名 `serial_writer` 維持。
- `RxParser` の再同期が健全: 各ループで最低1byte or 1フレーム消費 → 停止性あり。CRC-16/Modbus が encode/decode 一致。
- 0N 初期保証(`thrust_` ゼロ初期化 → 初回 timer で 0N 送信)。切断時 reconnect/backoff。
- fd アクセスは `fd_mutex_` で保護。read は nonblocking + EAGAIN 時 sleep で TX を止めない。
- `/thruster_command` = Newton を維持、bridge は再スケールせず memcpy(単位不変条件 OK)。
- bms は tty を開かず `micon/bms_cells`→`bms/cell_voltages` + 低電圧 diagnostics。

## 指摘(いずれも非ブロッカー / 主に運用前の注意)
1. [運用注意] SW E-stop 未接続: `serial_writer::emg_cb` は依然 `flags_.emergency=false` 固定
   (HW ボタン前提の既存仕様、Sol も指摘)。通電前に「本当に HW E-stop のみで良いか」をユーザ確認推奨。
2. [軽微] `ekf_original.yaml` の "kinematics" コメント書換は無関係語の誤検出掃除(robot_localization 定型文)。害はないが不要な差分。
3. [情報] real_bringup は zed/livox/wit/um982 等を build/install しないと実起動不可(構造は検証済)。
   艇上での実起動前に各 sensor pkg のビルドと、msg_MID360/wit/um982 の device 引数実値を確認すること。
4. [軽微] timer 書込と rx 再接続の間に fd の TOCTOU が理論上あるが、50ms 周期・同一ポートで実害ほぼ無し。許容。
5. [情報] 実行環境は ROS2 **Jazzy**(Humble 未導入)。CLAUDE 記載/一部コメントの "Humble" と乖離。ビルド/テストは Jazzy で成功。

## 判定
**合格(再修正不要)**。受け入れ条件 AC1〜AC3/AC5〜AC7 を満たす。AC4(bringup dry-run)は構造レベルで検証済、
実 HW 一括起動は艇上工程。指摘1(E-stop)はユーザ確認事項として引き継ぐ。
