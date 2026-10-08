# Njord workspace ToDo

この一覧は `test07089` を直接 master に取り込まず、機能単位の小さな
PR に分けるための実行チェックリストである。状態や計測値は実機試験の
記録へ残し、このファイルには配線図・シリアル番号・位置情報を載せない。

## 最優先: Task 1-1 相当の安全確認

- [ ] 使用する miniPC/Jetson/地上PC の commit、ROS_DOMAIN_ID、時刻同期と
  ネットワーク到達性を記録する。
- [ ] 機体を支持・固定し、外部電源遮断と非常停止を確認する。最初の起動は
  必ず `enable_thruster:=false` とし、駆動系を接続しない。
- [ ] MID360S の LiDAR/IMU、GLIM の `/odom`、UM982 のraw fixを別々に観測する。
- [ ] `map -> odom -> base_link -> livox_frame` が単一の発行者で連続することを
  `view_frames` と `tf2_echo` で確認する。重複TFやGNSS/EKFの不連続があれば
  Nav2・経路追従へ進まない。
- [ ] Task 1 waypoints が `map` で妥当な位置・順序に表示されることを、停止中に
  確認する。詳細な観測手順は `Docs/task1_1_today_verification.md` に従う。
- [ ] 実機で走行する前に rosbag、担当者、外部停止操作、試験範囲と中止条件を
  記録する。これは自律走行の許可ではない。

## 3輪オムニMD10C（Task 1-1とは別の駆動系準備）

- [x] 専用のhost profile、3輪URDF、UART type `0x02` firmwareを追加する。
- [x] 水上機用の type `0x01` / 4×force[N] profileを既定として残す。
- [x] host・firmwareの双方で duty を `abs <= 0.50` に制限し、無効入力・
  watchdog timeout・soft emergency で全chをゼロにする。
- [ ] 実機の外部電源遮断・非常停止経路を確認し、MD10C profileの独立E-stopと
  誤認しない。
- [ ] 各輪を浮かせた状態で、低duty・1輪ずつ、実配線の極性を確認する。
  `reverse` と firmware の `kDirectionInverted` は同じ物理反転を二重に設定しない。
- [ ] 校正記録をレビュー後にのみ、host YAML の
  `safety.actuator_configuration_confirmed` と firmware の
  `kMotorOutputEnabled` の二つのlockを解除する。
- [ ] 外部位置計測を使って直進・横移動・旋回を低速で記録し、open-loop duty係数を
  校正する。エンコーダがないため、`cmd_vel` を物理速度と扱わない。

3輪を起動する場合は、次の三つを必ず同時に選ぶ。単独に切り替えない。

```text
thruster_config_file:=<thruster_driver>/config/omni_md10c3.yaml
thruster_robot_description_file:=<robot>/urdf/omni_3wheel.urdf
thruster_serial_config_file:=<micon_driver_fd>/config/omni_md10c3.yaml
```

## test07089 の整理

- [ ] masterを基準に、各統合sliceを独立branchとdraft PRにする。
- [ ] CIが通り、影響するlaunch/configのレビューが済むまでmasterへmergeしない。
- [ ] 生成物、ローカルAI設定、個人環境ファイル、巨大vendor/modelは機能PRから外し、
  明示的な更新理由・ライセンス・再現方法を付けて別扱いにする。
- [ ] Project権限が付与されたら、`統合基盤`、`安全/駆動`、`位置推定`、`Task 1`、
  `Task 2以降`、`知覚`、`実機検証`をProject itemにする。

分割順序と各sliceの受入条件は [plan.md](plan.md) を参照する。
