# mission_manager

`mission_manager` は、NJORD タスクを一つずつ実行する正規のミッション API です。タスク設定は [config/task_registry.yaml](config/task_registry.yaml) で管理します。

```bash
ros2 launch mission_manager mission.launch.py active_nav2_profile:=task1
```

## 操作

`njord-task` は `/mission/run_task`、`/mission/start_task`、`/mission/stop_task` を呼ぶ CLI です。

```bash
ros2 run mission_manager njord-task list
ros2 run mission_manager njord-task check task1_skip_1_1
ros2 run mission_manager njord-task start task1_skip_1_1 --auto
ros2 run mission_manager njord-task status
ros2 run mission_manager njord-task stop
```

`check` は経路と Nav2 プロファイルを検証する dry-run です。`start --auto` は `/control/set_mode` に AUTO を要求しますが、実際の開始は `control_manager` が `/control/state` で許可するまで待ちます。
`njord-task`は`mission_manager`パッケージのconsole scriptです。単体コマンドとしてPATHに登録されていない環境では、上記のように`ros2 run mission_manager njord-task ...`で実行します。

## 主な入出力

| 種別 | 名前 | 用途 |
| --- | --- | --- |
| Action | `/mission/run_task` | タスクを実行／dry-run |
| Service | `/mission/start_task`, `/mission/stop_task` | タスク開始・停止 |
| Service | `/mission/list_tasks`, `/mission/get_status` | 登録済みタスク・状態の取得 |
| Publish | `/mission/status` | 実行状態、進捗、AUTO 阻害理由 |
| Publish | `/mission/last_result` | IDLEに戻っても保持する直前の終了結果（transient local） |
| Publish | `/mission/plan` | 投影後の全ウェイポイント（sim・GUI確認用） |
| Publish | `/mission/task_ready`, `/mission/task_requirements_ready` | `control_manager` への AUTO readiness |
| Subscribe | `/control/state` | AUTO 許可状態 |

`/mission/run_task is unavailable` は、このパッケージの `mission_manager_node` が起動していない、または同じ ROS domain にいないことを示します。Nav2 未起動は、開始後に Nav2 goal が拒否される別の問題です。


## センサなしの実行

[Hardware-free mission simulation](../../sim/njord_sim/README.md) の起動手順で
Mission / Control / Nav2を通した試験ができます。`task1_2` はGPS3から開始し、
GPS3の到達成功後に既存の長い仮想壁を有効にします。Task4は現在の
`staged_docking` 経路を使用し、古いTask4 composite executorは使用しません。

停泊は新鮮なlocal/global odometryで位置・方位・速度・角速度を確認し、
2秒静止した後に連続10秒（通常）/ 5秒（縦列）を数えます。動いたらリセットし、
odometry欠落時は進行しません。既定値は仮設定で、次のROSパラメータで変更できます。

| パラメータ | 既定値 |
| --- | --- |
| `docking_stationary_position_tolerance_m` | 0.8 |
| `docking_stationary_yaw_tolerance_rad` | 10度相当 |
| `docking_goal_yaw_tolerance_rad` | 5度相当 |
| `docking_stationary_speed_mps` | 0.1 |
| `docking_stationary_yaw_rate_radps` | 0.05 |
| `docking_odometry_timeout_sec` | 0.5 |
| `docking_settle_time_sec` | 2.0 |
| `docking_wait_timeout_sec` | 120.0 |

これは停泊中の監視幅です。simのNav2到達条件はXY 0.15mとし、評価器は
別途、真値で船体全体の枠内収容と接触ゼロを要求します。
