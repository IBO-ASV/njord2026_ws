# 制御とミッションの責務

現行API・設定の正本は [mission_manager](../../src/mission/mission_manager/README.md)、
[control_manager](../../src/control/control_manager/README.md)、
[robot](../../src/robot/README.md) です。旧実装計画と移行前baselineは統合しました。
旧仕様の詳細は整理前 commit `74fe7ec6` に残っています。

- 端末別bringupがハードウェアと常駐ノードを所有し、Mission Managerがタスク実行を所有します。
- CLIとGUIは同じtyped mission APIを使います。AUTO要求・許可・実行状態を分離します。
- 安全判定はcontrol_managerのpolicy、出力選択はcommand arbiterが担います。
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
