# velocity_autotuner — 前後速度ゲインの自動調整

前後速度（surge）のFF → P → Dゲインを、指定した範囲内で順に探索するROS 2パッケージです。
起動直後は`IDLE`で、開始操作まで走行しません。開始すると船が動きます。
横移動・旋回のゲインや、スラスター個別の推力比率は調整しません。

## 起動と操作

通常のAGX bringupと同じworkspace・ROS環境を使います。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to velocity_autotuner thruster_controller motion_supervisor --symlink-install
source install/setup.bash
scripts/operations/run_agx_pair.sh vessel
```

別端末で同じworkspaceから待機中の調整器を起動します。

```bash
scripts/operations/run_agx_velocity_tuner.sh
```

通常のAGX環境を設定済みなら、直接起動もできます。

```bash
ros2 launch velocity_autotuner velocity_autotuner.launch.py
```

FoxgloveのMission Control「速度ゲイン調整」で、目標速度・速度上限・推力上限を設定し、
AUTO要求・AUTO許可を確認して開始します。3値の既定値は0なので、未設定では開始できません。
手動で位置を直す場合は一時停止してMANUALへ切り替え、戻ったらAUTOで再開します。
終了操作はMANUALで行います。画面での手順は
[運用手順書](../../../docs/operations/velocity_gain_autotuning.md)を参照してください。

## 主要パラメータ

| パラメータ | 既定値 | 内容 |
|---|---|---|
| `target_speed_mps` | `0.0` | 目標前後速度。正の値が必要 |
| `max_test_speed_mps` | `0.0` | 速度上限。目標速度以上が必要 |
| `test_max_thruster_thrust_n` | `0.0` | 1基あたりの試験推力上限。正の値が必要 |
| `feedforward_bounds` / `p_bounds` / `d_bounds` | `[0.0, 80.0]` | FF/P/Dそれぞれの探索範囲 |
| `search_rounds` | `4` | 探索の絞り込み回数 |
| `settle_time_sec` / `steady_window_sec` | `25.0` / `8.0` | FFの整定待ちと定常評価時間 |
| `step_duration_sec` / `pre_step_stop_sec` | `18.0` / `3.0` | P/Dの評価時間と試行前のゼロ指令時間 |
| `odom_topic` | `/odometry/filtered/local` | 前後速度の入力。`child_frame_id`が`base_link`のOdometry |
| `force_topic` | `/debug/current_force` | 推力入力。Float32MultiArrayの4要素 |
| `controller_node` | `/thruster_controller_node` | パラメータ操作先 |
| `output_dir` | `~/njord_tuning` | CSV・YAMLの保存先 |

Mission Controlは3つの試験上限を`/velocity_autotuner/set_parameters_atomically`で設定します。
有効なセッション中は設定変更を拒否します。探索範囲などは起動時のROSパラメータで指定してください。
目標速度はコントローラの`input_scaling.max_linear_x`も超えられません。

## ROSインターフェース

以下はすべて`std_srvs/srv/Trigger`サービスです。応答の成功は要求の受理を示し、
非同期処理の完了は`/velocity_tuning/status`（String）で確認します。

| サービス | 操作 |
|---|---|
| `/velocity_tuning/start` | 新鮮なオドメトリとAUTO許可を確認して開始 |
| `/velocity_tuning/pause` | 一時停止。手動操作には別途MANUALへの切替が必要 |
| `/velocity_tuning/resume` | AUTO許可を確認し、現在の候補を最初から再試行 |
| `/velocity_tuning/stop` | MANUALで終了・設定復元を要求 |

出力は`/cmd_vel_tune`（Twist）と`/control/tuning_active`（Bool）です。
通常のcommand arbiterが転送し、非常停止・MANUAL・SBUSの優先順位とAUTO許可を維持します。
非常停止、手動介入、オドメトリ・推力の途絶、速度超過で`PAUSED`となり、調整指令をゼロにします。

状態は`IDLE` → `PREPARING` → `RUNNING` → `FINALIZING` → `COMPLETE`と進みます。
終了要求後は`RESTORING` → `IDLE`です。復元失敗時は`PAUSED`のまま理由を表示するため、
MANUALを維持して原因を解消し、終了を再試行してください。
パラメータ通信が処理中の場合は、その応答完了後に終了を再試行します。

## 結果と設定の復元

FFは定常速度誤差、P/Dは速度誤差の積分・オーバーシュート・推力変化を評価します。
試験中は前後速度のIゲインを0にし、既存の上限を超えないよう試験推力上限を設定します。
選定したFF/P/Dの反映成功とYAML保存を確認してから`COMPLETE`になります。

- `velocity_tuning_<UTC時刻>.csv`：候補ごとの評価結果
- `velocity_gains_<UTC時刻>.yaml`：選定したFF/P/D

終了時に推力上限とIゲインを復元します。完了済みなら選定FF/P/Dを残し、途中終了なら元のFF/P/Dも復元します。
YAMLは選定値の記録であり、標準設定やコントローラのパラメータファイルには自動反映しません。
プロセス強制終了では復元サービス処理が走らないため、終了操作と状態確認を行ってください。

## 検証

ROS環境とworkspaceをsourceした後、回帰テストを実行できます。

```bash
python3 -m pytest -q src/control/velocity_autotuner/test
```

終了時の選定ゲイン反映、YAML読込、復元失敗・再試行、一時停止と非同期応答の競合を検証しています。
実船での調整性能は未検証です。
