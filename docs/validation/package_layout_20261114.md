# 役割別package再編の検証（2026-11-14）

[責務・移行表](../architecture/control_mission.md#2026-11-14-役割別packageへの再編)と
[検証要約](fixtures/package_layout_20261114/summary.json)を参照。

- ROS 2 Jazzyの本workspaceで、分割した型・実装・consumerの39 packageをビルド。
- 旧`njord_interfaces`から移した18定義を旧commitと比較し、型参照のpackage名以外の
  フィールド・定数・順序が一致。未使用のBms/Thruster定義は廃止。
- Python 241件成功。sim、mission、motion supervisor、operator gateway、waypoint
  transformer、buoy obstacle、robot/test bringupの既存テストを実行。
- 8 packageのC++/lint結果: 76件、失敗0、エラー0、skip 14（amentのcppcheck 2.13.0性能問題によるskip）。
- [7つのlaunch引数展開](fixtures/package_layout_20261114/launch_checks.json)に成功。
- [MMGでのTask 1.2 / Task 2 head-on](fixtures/package_layout_20261114/suite.json)は
  両方成功。mission完了と終了時の速度指令ゼロを確認。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp python3 -m pytest -q \
  src/sim/njord_sim/test src/mission/mission_manager/test \
  src/control/motion_supervisor/test src/bringup/robot/test \
  src/operator/operator_gateway/test src/navigation/waypoint_transformer/test \
  src/navigation/buoy_obstacle_publisher/test src/bringup/test_bringup/test
ROS_DOMAIN_ID=182 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  colcon test --packages-select joystick_control sbus_driver emergency_stop_selector \
  critical_link status_lights thruster_controller dutyed_tf_pub_with_disturbance diagnostic_monitors
python3 scripts/simulation/run_sim_suite.py --tasks task1_2,task2 \
  --truth --seeds '' --workers 2 --domain-base 150 --timeout 300 \
  --output artifacts/package_layout_migration/smoke-recheck
```

初回の検証で、移動後のテスト参照パス、状態灯launch/configのファイル名、
長くなったnamespaceの整形、task2_perceptionの不足依存を修正した。
ZED契約テストの古いSDK内コメント参照は、実際の型変換helperのクラスID転送を確認する形へ更新。
既存の制御則・topic名・UART/critical-link packetは変更していない。

全task・全noise seedの再試験、実機/serial送信、Jetson/miniPC間の通信試験、GitHub CIは
この検証には含まない。実機側・地上側とも新ROS型を同じ版で再ビルドする。
過去bagには記録時の環境を使う。
