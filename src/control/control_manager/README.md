# control_manager

`control_manager` は、操縦モードと最終速度指令を一元管理する ROS 2 パッケージです。

```bash
ros2 launch control_manager control.launch.py
```

## ノード

| ノード | 役割 |
| --- | --- |
| `mode_manager` | `/control/set_mode` で MANUAL / AUTO の要求を受け付け、`/control/requested_mode` を publish |
| `safety_supervisor` | AUTO 許可を判定し、正規状態 `/control/state` を publish |
| `command_arbiter` | `/control/state` に従い `/cmd_vel_manual` または `/cmd_vel_nav` を `/cmd_vel` へ送る。条件未達・指令期限切れではゼロ指令 |

## AUTO の状態確認

`/control/state` の唯一の publisher は `safety_supervisor` です。`auto_permitted` が `true` でなければ AUTO 指令は通りません。理由は `inhibit_reasons` を確認してください。

```bash
ros2 topic echo --once /control/state
ros2 topic info -v /control/state
```

AUTO の許可条件は [config/control_policy.yaml](config/control_policy.yaml) にあります。非常停止、Nav2・タスク準備、Health Signal、およびタスク固有の readiness を評価します。
