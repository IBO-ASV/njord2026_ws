# thruster_driver

> **Profile boundary:** the legacy watercraft profile is four-element Newton
> output, but the independent `omni_wheel_duty` profile is three-element
> signed duty output.  Do not read the legacy four-thruster description below
> as applying to the MD10C configuration; see the profile contract at the end.

`cmd_vel` と feedback odometry から船体 wrench を計算し、URDF 上の
4スラスタ配置に配分して `/thruster_command` へ publish します。

- 入力:
  - `cmd_vel` (`geometry_msgs/msg/Twist`)
  - feedback odometry (`nav_msgs/msg/Odometry`)
  - または `thruster_command` (`std_msgs/msg/Int16MultiArray`) を直接受ける
- 出力:
  - `/thruster_command` (`std_msgs/msg/Float32MultiArray`, 4要素, Newton)

`/thruster_command` は常に Newton です。実機では `micon_driver_fd/serial_writer`
がこの値を再スケールせず、そのまま 4×`float32` として ESP32 へ送ります。

## ポイント

- CAN や per-thruster ROS topic への出力は持ちません。
- URDFはスラスタの固定poseだけを持ち、推力方向・反転・ゲインはconfigで切替えます。
- スラスタ別の出力補正は `static_map.thrusters` で設定します。
- `control.dob.enable` でP+DOBとP-onlyを切替できます。
- 起動時、cmd timeout、feedback timeout時は 0N を publish します。

## 信号の流れ (cmd_vel モード)

```
cmd_vel (Twist)
  └─ input_scaling で目標速度に clamp
      └─ P 制御 (+ 任意で DOB) → wrench [surge, sway, yaw] (N, N, N·m)
          └─ control.max_*_wrench で wrench を clamp
              └─ allocation (最小二乗) → 各スラスタの正規化指令 [-1, 1]
                  └─ static_map (deadzone / gain / offset)
                      └─ 正規化指令 × thrusters.max_thrust → Newton
                          └─ /thruster_command (Float32MultiArray, 4要素, N)
```

`/thruster_command` は常に Newton で、`serial_writer` が再スケールせず ESP32 へ渡します。

## 最大スラスト力の調整方法

各スラスタの最大出力は **`thrusters.max_thrust`** (単位: N, スラスタ数と同数の配列) で決まります。
この値は 2 か所で使われます。

1. **allocation の推力ゲイン** — 各スラスタが単位指令あたり出せる力として推力配分行列に入ります。
2. **最終スケール** — 正規化指令 `[-1, 1]` に掛けて Newton に変換します (`node.cpp` の `publishCommands`)。

したがって 1 基あたりの出力は `±max_thrust [N]` で飽和します。既定値は
BlueRobotics T200 @16V の最大推力 **≈51.5N** です。

- スラスタや電源電圧を変えて物理的な上限が変わったとき → `thrusters.max_thrust` を実測/データシート値に更新する。
- 弱い/強いスラスタを混在させるとき → 配列の各要素を個別に設定する (allocation が推力差を考慮して配分します)。

### 注意: 実際に最大推力へ届くには wrench clamp も必要

`thrusters.max_thrust` を上げても、コントローラが要求する wrench が
`control.max_surge_wrench` / `max_sway_wrench` / `max_yaw_wrench` で頭打ちになっていると
各スラスタは `max_thrust` まで届きません。full-scale の推力を出したい場合は
**両方**を十分大きくしてください。

- 自律 (cmd_vel from 制御) 時は `control.p.*` ゲインと `input_scaling.max_*` の組み合わせで
  wrench の大きさが決まります。
- 手動操作時は `manual_control.launch.py` の full-scale stick/button 入力が
  約 2N/スラスタになるよう `max_*_wrench` が調整されています (`simple_manual/README.md` 参照)。
  手動の最大推力を上げたいときは `max_*_wrench` を上げます。

## config パラメータの意味

`config/config.yaml` (`thruster_driver_node` 名前空間)。

| パラメータ | 単位 | 意味 |
| --- | --- | --- |
| `input_mode` | - | `cmd_vel` (速度指令→制御) か `duty_array` (正規化指令を直接受ける) |
| `duty_resolution` | - | `duty_array` モードで Int16 指令を正規化するときの分母 (例: 1000 → ±1000 が ±1.0) |
| `input_scaling.max_linear_x` | m/s | `cmd_vel.linear.x` を clamp する上限 (surge 目標速度) |
| `input_scaling.max_linear_y` | m/s | `cmd_vel.linear.y` を clamp する上限 (sway 目標速度) |
| `input_scaling.max_angular_z` | rad/s | `cmd_vel.angular.z` を clamp する上限 (yaw 目標角速度) |
| `control.rate_hz` | Hz | 制御ループ (control timer) の周期 |
| `control.feedback_timeout_sec` | s | odometry フィードバックが途切れたと判定するまでの時間 |
| `control.stop_on_feedback_timeout` | bool | feedback timeout 時に 0N を出すか (false なら P 制御を継続) |
| `control.p.surge` / `sway` / `yaw` | N/(m/s), N·m/(rad/s) | 速度誤差に対する P ゲイン。wrench = P × (目標 − 実測) |
| `control.max_surge_wrench` | N | surge 方向 wrench の clamp 上限 |
| `control.max_sway_wrench` | N | sway 方向 wrench の clamp 上限 |
| `control.max_yaw_wrench` | N·m | yaw 方向 wrench (トルク) の clamp 上限 |
| `control.dob.enable` | bool | 外乱オブザーバ (DOB) の有効/無効。false なら P-only |
| `control.dob.observer_gain` | - | DOB 推定外乱の反映ゲイン |
| `control.dob.filter_tau_sec` | s | DOB の 1 次 LPF 時定数 |
| `control.nominal.mass` | kg | 公称質量 (DOB の慣性項に使用) |
| `control.nominal.iz` | kg·m² | 公称ヨー慣性モーメント (DOB に使用) |
| `control.nominal.damping.linear.*` | - | 公称の線形減衰係数 (surge/sway/yaw、DOB に使用) |
| `control.nominal.damping.quadratic.*` | - | 公称の 2 次減衰係数 (速度の 2 乗に比例、DOB に使用) |
| `allocation.regularization_lambda` | - | 推力配分の最小二乗に加える正則化項。大きいほど指令が滑らか・小さいほど厳密 |
| `safety.watchdog_timeout_sec` | s | `cmd_vel` が途切れたと判定するまでの時間。超過で 0N |
| `static_map.deadzone_pos` / `deadzone_neg` | - | 正/負側の正規化指令デッドゾーン (この絶対値以下は 0) |
| `static_map.thrusters.forward_gain` | - | 正指令側のスラスタ別ゲイン (配列) |
| `static_map.thrusters.reverse_gain` | - | 負指令側のスラスタ別ゲイン (配列) |
| `static_map.thrusters.offset` | - | スラスタ別の正規化指令オフセット (配列) |
| `thrusters.ids` | - | スラスタ識別子 (配列)。他の全配列の長さの基準 |
| `thrusters.links` | - | 各スラスタ pose を読む URDF link 名 (配列) |
| `thrusters.angle_rad` | rad | 各スラスタの推力方向 (base_link 基準、配列) |
| `thrusters.reverse` | bool | 各スラスタの推力符号反転 (配線・プロペラ向きの補正、配列) |
| `thrusters.max_thrust` | N | **各スラスタの最大推力**。上記「最大スラスト力の調整方法」を参照 (配列) |
| `topics.cmd_vel` | - | 速度指令の入力 topic 名 |
| `topics.duty_array` | - | `duty_array` モードの入力 topic 名 |
| `topics.feedback_odometry` | - | フィードバック odometry topic 名 |
| `topics.sim_thruster_command` | - | 出力する `/thruster_command` topic 名 |

> スラスタ関連の配列 (`ids`, `links`, `angle_rad`, `reverse`, `max_thrust`,
> `static_map.thrusters.*`) は必ず `thrusters.ids` と同じ長さにしてください。
> 長さが不一致だとノード起動時に例外で停止します。
# Profile-specific output contract

`/thruster_command` is **not always Newton**.  The original marine
`wrench_allocation` / `force_newton` profile publishes exactly four `float32`
force values in N.  The independent `omni_wheel_duty` profile in
`config/omni_md10c3.yaml` publishes exactly three signed duty ratios and must
only be paired with `micon_driver_fd/config/omni_md10c3.yaml`.

The serial boundary now rejects rather than pads or truncates malformed input:
`esc4_force` requires exactly four finite values, and `md10c3_duty` requires
exactly three finite values.  Existing workspace producers were checked:
the normal marine `thruster_driver` configuration has four IDs and its
`Float32MultiArray` producer therefore emits four values; the three-wheel
profiles select their dedicated three-element configuration.  External or
ad-hoc publishers must meet the same exact-length contract.

<!-- End of profile-specific contract. -->
