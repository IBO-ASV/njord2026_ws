# mission_manager

`mission_manager` は、NJORD タスクを一つずつ実行する正規のミッション API です。タスク設定は [config/task_registry.yaml](config/task_registry.yaml) で管理します。

```bash
ros2 launch mission_manager mission.launch.py active_nav2_profile:=task1
```

## 操作

`njord-task` は `/mission/run_task`、`/mission/start_task`、`/mission/stop_task` を呼ぶ CLI です。

```bash
njord-task list
njord-task check task1_skip_1_1
njord-task start task1_skip_1_1 --auto
njord-task status
njord-task stop
```

`check` は経路と Nav2 プロファイルを検証する dry-run です。`start --auto` は `/control/set_mode` に AUTO を要求しますが、実際の開始は `control_manager` が `/control/state` で許可するまで待ちます。

## 主な入出力

| 種別 | 名前 | 用途 |
| --- | --- | --- |
| Action | `/mission/run_task` | タスクを実行／dry-run |
| Service | `/mission/start_task`, `/mission/stop_task` | タスク開始・停止 |
| Service | `/mission/list_tasks`, `/mission/get_status` | 登録済みタスク・状態の取得 |
| Publish | `/mission/status` | 実行状態、進捗、AUTO 阻害理由 |
| Publish | `/mission/task_ready`, `/mission/task_requirements_ready` | `control_manager` への AUTO readiness |
| Subscribe | `/control/state` | AUTO 許可状態 |

`/mission/run_task is unavailable` は、このパッケージの `mission_manager_node` が起動していない、または同じ ROS domain にいないことを示します。Nav2 未起動は、開始後に Nav2 goal が拒否される別の問題です。
