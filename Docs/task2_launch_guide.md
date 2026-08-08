# Task 2 launch 構成

Task 2で利用する入口は、用途ごとに次の4つです。

| 用途 | 起動コマンド | センサー・スラスタの扱い |
|---|---|---|
| 実機自動運転 | `ros2 launch robot task2_autonomy.launch.py` | 起動しない。すでに動作している `simple_manual/manual_control.launch.py` のLiDAR、自己位置、TF、command_arbiterを利用する。 |
| 認識単体デバッグ | `ros2 launch task2_perception task2_perception.launch.py` | センサーは起動しない。入力済みの `/livox/lidar` を利用する。追跡には別途 `classical_pipeline.launch.py` を起動する。 |
| rosbag認識デバッグ | `ros2 launch robot task2_bag_perception.launch.py bag_path:=/path/to/bag` | 実機センサー・スラスタは起動しない。bagを再生してLiDAR認識・追跡を検証する。 |
| MPPIシミュレーション | `ros2 launch task2_sim task2_sim.launch.py` | 他船を認識済みと仮定し、相手船TFからMPPI・FollowPath・`/cmd_vel` 理想追従運動までを検証する。LiDAR認識器、スラスタドライバ、推力配分・船体力学は起動しない。 |

実機では、先に `simple_manual/manual_control.launch.py` を起動し、手動確認後に
`task2_autonomy.launch.py` を起動して自動モードへ切り替えます。

`task2_autonomy.launch.py` は、Task 2用LiDAR認識、他船追跡、MPPI、Nav2 FollowPath、
velocity_smootherをまとめて起動します。ハードウェアドライバ、センサー、command_arbiter、
スラスタドライバは一切起動しません。

`navigation_launch_task2.py` は、実機自動運転とシミュレーションから共通利用する内部launchです。
単独で起動する運用入口ではありません。
