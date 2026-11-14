# 制御とミッションの責務

現行API・設定の正本は [mission_manager](../../src/mission/mission_manager/README.md)、
[motion_supervisor](../../src/control/motion_supervisor/README.md)、
[robot](../../src/bringup/robot/README.md) です。旧実装計画と移行前baselineは統合しました。
旧仕様の詳細は整理前 commit `74fe7ec6` に残っています。

- 端末別bringupがハードウェアと常駐ノードを所有し、Mission Managerがタスク実行を所有します。
- CLIとGUIは同じtyped mission APIを使います。AUTO要求・許可・実行状態を分離します。
- 安全判定はmotion_supervisorのpolicy、出力選択はcommand arbiterが担います。
- 起動直後の自動目標送信・再起動後の自動resumeは行いません。
- 旧task launchは互換用で、旧グラフの起動には明示的な引数が必要です。
- Nav2 profileのframe・controller・costmap構成差は任意parameter変更で代用しません。

## 残る確認事項

旧計画に記載された全実機試験が完了したとは確認できません。以下の検証要求を残します。
特にcancel timeout、遅延callback、通信断、プロセス再起動、profile切替のrollbackを
実機投入前に確認してください。今回の整理は動作や安全判定を変更しません。

## 17. Test Plan

### 17.1 Unit tests

- Every legal and illegal mission state transition.
- Every control truth-table combination.
- Policy defaults, ON/OFF, unknown keys, and unknown health references.
- Task registry schema and file references.
- Waypoint uniqueness, finite values, stage references, and empty routes.
- Request-id idempotency and execution-id stale callback handling.
- Task 3 stage transitions, retries, wait cancellation, and cleanup.

### 17.2 Integration tests with fake Nav2

Create a fake `NavigateThroughPoses` server that can be scripted to:

- remain unavailable;
- reject a goal;
- accept and succeed;
- accept and abort;
- delay feedback/result;
- accept cancellation;
- ignore cancellation for timeout testing.

Test:

- start, feedback, success;
- cancel before acceptance and during execution;
- E-stop during navigation;
- required heartbeat loss;
- command freshness timeout;
- Mission Manager restart;
- duplicate and competing task requests;
- late result from a prior execution.

### 17.3 Launch tests

- Exactly one Mission Manager on miniPC/standalone.
- Exactly one canonical `/cmd_vel` publisher.
- Role bringup does not submit a Nav2 task goal.
- Jetson role does not start miniPC hardware/control owners.
- Ground role does not publish canonical joystick/control Topics directly.
- Standalone composition does not duplicate networking or hardware nodes.

### 17.4 Hardware test order

1. Dry-run only.
2. Thruster output physically disabled.
3. MANUAL/AUTO request and inhibit display.
4. E-stop during pending and running task.
5. Low-output Task 1.
6. Ground communication loss.
7. GNSS/localization/heartbeat fault injection.
8. Task cancellation.
9. MANUAL-mediated task switching.
10. Task 2 and Task 3.

Record rosbag data and Mission/Control/Health status for each safety test.


## 2026-11-14: 役割別packageへの再編

ROS標準型のJoy/Twist/Odometry、Float32/Float32MultiArray、DiagnosticArrayなどは
そのまま使う。未使用の旧Bms/Thruster定義は削除した。状態コード、要求/実行ID、
阻害理由、方位ブイの意味などを持つ型は、汎用文字列へ置き換えず契約を維持する。

| 旧package/配置 | 現行の責務と配置 |
|---|---|
| njord_interfaces | `interfaces/{control,mission,health,perception,runtime,operator}_interfaces` |
| njord_platform | `build_tools/sensor_build_support`: backend検出とmanifest |
| simple_manualのJoy変換 | `input/joystick_control` |
| simple_manualのSBUS受信 | `driver/rc/sbus_driver`（出力はsensor_msgs/Joy） |
| simple_manualのsoft EMG選択 | `safety/emergency_stop_selector` |
| simple_manualの旧仲裁 | `control/legacy_command_mux`（legacy bringup専用） |
| simple_manualの地上局heartbeat | `communication/ground_station_heartbeat` |
| simple_manualの機器試験 | `tools/thruster_tools` |
| simple_manualの全機起動 | `bringup/manual_bringup` |
| control_manager | `control/motion_supervisor`: モード・policy・最終出力源の決定 |
| driver/micon/thruster_driver | `control/thruster_controller`: 速度PI・推力配分・静特性補正 |
| critical_link | `communication/critical_link`: transportとpacket codec |
| alert_lamp | `operator/status_lights`: 状態表示の判断と状態灯出力 |
| mission_manager内runtime | `system/runtime_manager`: Nav2 process/profileの所有 |
| mission_manager内operator adapter | `operator/operator_gateway`: typed要求の検証・dispatch |
| behavior/system_manager | `system/parameter_gateway`: 旧remote parameter action adapter |
| mission/waypoint_transformer | `navigation/waypoint_transformer`: 座標合わせ |
| robot、test_bringup | `bringup/`へ配置。ROS package名は維持 |
| foxglove_image_preview | `gui/`へ配置。ROS package名は維持 |

`mission_manager`にはtask実行、経路/停泊状態機械、mission固有のfailsafeを残す。
runtime/operator adapterはmissionの実装をimportせず、各役割のinterfaceだけに依存する。
各stackの合成は`robot/mission_stack.launch.py`へ移し、task実行packageから分離した。
`motion_supervisor`は速度制御器ではなく制御権限の所有者、`thruster_controller`は
ハードウェアUART driverではなく速度制御・配分である。UARTは従来どおり`micon_driver_fd`が担う。

`sensor_build_support`はCheckLanguage/CUDAToolkit等の標準CMake検出に、
workspace共通のbackend/profile方針を加える薄い共有層。未使用の生成C++headerは廃止し、
CMake extrasとplatform.yaml/envだけを公開する。既存のNJORD_*環境変数は維持する。
`parameter_gateway`は旧API向けの任意adapterであり、現行runtimeの所有者ではない。

### 移行上の契約

- 制御則、状態コード、topic名、要求/実行ID、critical-link packetとUART protocolは変更しない。
- ROSIDL型のpackage名は変わる。全端末のconsumerを同じ版で再ビルドしてから使用する。
- bringup引数の`enable_control_manager`は`enable_motion_supervisor`へ変更。
- `ros2 launch simple_manual manual_control.launch.py`は
  `ros2 launch manual_bringup manual_control.launch.py`へ変更。
- `ros2 launch mission_manager mission.launch.py`は
  `ros2 launch robot mission_stack.launch.py`へ変更。
- `njord-critical`の配布元は`operator_gateway`。`njord-task`は引き続き`mission_manager`。
- 過去のbag/数値fixture/レビュー記録は変換しない。旧型を必要とする再生には記録時の版の環境を使う。
- 作業環境の旧build/installと不要になったentry pointは、
  `artifacts/package_layout_migration/`へ退避した。新規cloneでは通常のJazzy buildを行う。

役割ごとのunit test、launch資産・依存関係、ROSIDL生成、hardware-free missionを確認する。
実機bringup・serial送信・GitHubへのpushはこの再編の検証には使わない。
