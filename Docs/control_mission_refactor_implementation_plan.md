# Control, Health, and Mission Runtime Refactor Plan

## 1. Purpose

This document is the implementation handoff for reorganizing NJORD's control-mode,
heartbeat/health, and task-launching logic.

The target operator workflow is:

```bash
# Start each computer once according to its role.
ros2 launch robot minipc_bringup.launch.py
ros2 launch robot jetson_bringup.launch.py
ros2 launch robot ground_pc.launch.py

# Later, select and run a task without relaunching hardware bringup.
ros2 action send_goal /mission/run_task njord_interfaces/action/RunTask \
  "{task_id: task1, request_auto_mode: true, dry_run: false, request_id: cli-001}" \
  --feedback
```

The same operation must be available from a GUI through small service adapters.
Starting a task must never bypass the emergency stop or the configured AUTO
permission policy.

This is an implementation plan, not a description of completed work. Implement it
incrementally and preserve compatibility until each replacement path is verified.

## 2. Current State and Problems

### 2.1 Control mode and command selection

- `src/simple_manual/src/joy_converter.cpp` owns the MANUAL/AUTO state and publishes
  `/system/operating_mode`.
- `src/simple_manual/src/command_arbiter.cpp` directly evaluates emergency stop,
  operating mode, `/autonomy/ready`, and command freshness.
- AUTO mode selection and AUTO command permission are not represented as separate
  states.
- The joystick configuration currently overlaps lamp and mode buttons:
  `yellow` and `auto_mode` both use button 2; `red` and `manual_mode` both use
  button 3.

### 2.2 Autonomy readiness

- `src/diagnostics/diagnostic_monitors/src/autonomy_supervisor.cpp` publishes
  `/autonomy/ready`.
- Its current readiness output only represents availability of the Nav2
  `NavigateThroughPoses` action server.
- Waypoint freshness and waypoint count are diagnostic warnings but are not
  readiness interlocks.
- The comments in `minipc_bringup.launch.py` still imply that fresh waypoints are
  part of readiness and therefore do not match the implementation.

### 2.3 Heartbeat and diagnostics

- Heartbeat generation is split between `robot/launch/heartbeat.launch.py`,
  `diagnostic_monitors/config/minipc_heartbeat.yaml`, and topic diagnostic profiles
  embedded in `robot/launch/diagnostics.launch.py`.
- `minipc_heartbeat.yaml` contains duplicate `minipc_heartbeat` keys. The first
  `minipc_heartbeat: []` does not disable the later list because the later mapping
  replaces it when loaded by PyYAML.
- Alert-lamp logic evaluates heartbeat, localization, RTK, global diagnostic
  severity, and control state itself.
- A global `DiagnosticArray` ERROR is treated as critical without a policy that
  identifies whether that diagnostic is relevant to propulsion safety.
- Some monitored conditions are display-only while others are expected to inhibit
  motion, but this distinction is not explicit in configuration.

### 2.4 Task launching

- `task1.launch.py`, `task2.launch.py`, and `task3.launch.py` each combine role
  bringup, Nav2, task-specific support nodes, delays, and automatic waypoint goal
  submission.
- Starting a different task currently implies starting another launch graph, which
  risks duplicate hardware owners and duplicate Nav2/control nodes.
- `waypoint_publisher_node.py` selects one task from a startup parameter and sends
  its Nav2 goal once when the action server appears.
- Task 3 already contains a useful staged state machine, but it is tied to the
  process lifetime and does not provide a top-level task action with consistent
  cancel/result semantics.
- `task1-2.launch.py` explicitly documents that its marker-driven route generator
  is not implemented.

### 2.5 Existing behavior/system manager

- `njord_interfaces/action/ConfigureSystem.action` and
  `behavior/system_manager` exist but are not a suitable task execution interface.
- `ConfigureSystem` accepts arbitrary string `parameter`/`value` pairs, does not
  represent task lifecycle, and accepts cancellation without checking it during
  execution.
- Do not expand this generic interface into the new mission API. Add typed mission
  interfaces instead.

## 3. Architectural Decisions

### 3.1 Role bringup is persistent

`<role>_bringup` owns persistent processes and hardware for that computer. It is
started once. Task selection must not start another role bringup.

- miniPC: vessel control, safety, Mission Manager, Nav2, GNSS/localization,
  thrusters, rear camera, BMS, alert lamp, health aggregation, networking.
- Jetson: LiDAR, front camera, GLIM, perception, Jetson health aggregation,
  networking.
- Ground PC: joystick, critical-link sender, heartbeat, video receivers, task
  client/GUI bridge, networking.
- standalone: compatibility composition of Jetson and miniPC roles without duplicate
  ownership.

### 3.2 Do not dynamically spawn ROS launch files from Mission Manager

Do not implement task switching by calling `ros2 launch` from a service callback or
by managing launch subprocesses inside Mission Manager. Process ownership, failure
recovery, duplicate nodes, and cancellation become difficult to make deterministic.

Instead:

- Keep common navigation/control processes resident.
- Represent task behavior as executors selected inside Mission Manager.
- Keep small task-support nodes resident but inactive, or convert them to lifecycle
  nodes/explicitly enabled nodes.
- Reconfigure Nav2 only while propulsion is inhibited and nodes are in a safe
  lifecycle state.

If a Nav2 task profile cannot initially be switched safely, reject that task with a
clear `CONFIGURATION_UNAVAILABLE` result. Do not silently start a second Nav2 graph.

### 3.3 Action is authoritative; services are adapters

A task is long-running, reports progress, and must be cancelable, so the canonical
API is `RunTask.action`. GUI-oriented services start or stop the same internal state
machine and return immediately after acceptance/rejection.

### 3.4 Separate requested mode, permission, and effective command source

Do not use one `manual`/`auto` string for all meanings.

- Requested mode: operator intent.
- AUTO permitted: result of the safety/control policy.
- Effective mode/source: what Command Arbiter is currently forwarding.
- Mission state: what task, if any, is being prepared or run.

### 3.5 Monitor every useful signal; configure inhibition separately

Turning a policy requirement OFF must not remove observability. For example, RTK can
be degraded and displayed while `require_rtk_fix: false` prevents it from inhibiting
AUTO. Heartbeat inventory defines how to observe a signal; control policy defines
whether the signal is required.

## 4. Target Runtime Graph

```text
Ground PC GUI / CLI
        |
        | RunTask action or StartTask/StopTask services
        v
  mission_manager ---------------------> /mission/status
        |                                      |
        | select executor                      +--> GUI / logger / lamp
        | configure task profile
        v
  task executor ---- NavigateThroughPoses ----> Nav2
        |                                       |
        +---------------- /task/plan             +--> /cmd_vel_nav
                                                        |
Joystick --> mode_manager --> requested mode             |
Health monitors -----------> health_supervisor           |
E-stop --------------------> safety_supervisor           |
Mission readiness ---------> safety_supervisor           |
                                  |                      |
                                  +--> /control/state <--+
                                             |
                                             v
                                      command_arbiter
                                             |
                                             v
                                          /cmd_vel
```

Only Command Arbiter publishes canonical `/cmd_vel`. Only the control stack owns the
canonical control state. Alert Lamp consumes state and does not independently derive
a competing safety decision.

## 5. Proposed Package and Directory Layout

```text
src/
├── control/
│   └── control_manager/
│       ├── include/control_manager/
│       │   ├── mode_manager.hpp
│       │   ├── safety_supervisor.hpp
│       │   └── command_arbiter.hpp
│       ├── src/
│       ├── config/
│       │   ├── control_policy.yaml
│       │   └── joystick.yaml
│       ├── launch/control.launch.py
│       ├── test/
│       ├── CMakeLists.txt
│       └── package.xml
│
├── mission/
│   └── mission_manager/
│       ├── mission_manager/
│       │   ├── mission_manager_node.py
│       │   ├── state_machine.py
│       │   ├── task_registry.py
│       │   └── executors/
│       │       ├── base.py
│       │       ├── waypoint_sequence.py
│       │       └── staged_docking.py
│       ├── config/
│       │   ├── task_registry.yaml
│       │   └── tasks/
│       │       ├── task1.yaml
│       │       ├── task2.yaml
│       │       └── task3.yaml
│       ├── launch/mission.launch.py
│       ├── test/
│       ├── package.xml
│       └── setup.py
│
├── diagnostics/
│   └── diagnostic_monitors/
│       └── config/heartbeat/
│           ├── minipc.yaml
│           ├── jetson.yaml
│           └── groundpc.yaml
│
├── njord_interfaces/
│   ├── action/RunTask.action
│   ├── srv/StartTask.srv
│   ├── srv/StopTask.srv
│   ├── srv/ListTasks.srv
│   ├── srv/GetMissionStatus.srv
│   ├── srv/SetControlMode.srv
│   ├── msg/MissionStatus.msg
│   ├── msg/TaskInfo.msg
│   ├── msg/ControlState.msg
│   ├── msg/HealthSignal.msg
│   └── msg/HealthState.msg
│
└── robot/
    └── launch/
        ├── minipc_bringup.launch.py
        ├── jetson_bringup.launch.py
        ├── ground_pc.launch.py
        ├── standalone_bringup.launch.py
        └── include/
            ├── navigation.launch.py
            ├── platform.launch.py
            └── networking.launch.py
```

Keep `robot` responsible for composition. Configuration owned by control, mission,
or diagnostics belongs to the corresponding package rather than `robot`.

## 6. ROS Interfaces

Exact names can be adjusted before Phase 1, but once generated they should be treated
as stable APIs.

### 6.1 `RunTask.action`

```text
# Goal
string task_id
bool request_auto_mode
bool dry_run
string request_id
---
# Result
uint8 SUCCEEDED=0
uint8 CANCELED=1
uint8 REJECTED=2
uint8 NOT_IMPLEMENTED=3
uint8 NOT_READY=4
uint8 SAFETY_INHIBITED=5
uint8 NAVIGATION_FAILED=6
uint8 CONFIGURATION_FAILED=7
uint8 INTERNAL_ERROR=255
uint8 result_code
string message
string execution_id
string[] final_inhibit_reasons
---
# Feedback
uint8 state
string task_id
string execution_id
string stage
float32 progress
string message
string[] inhibit_reasons
```

Do not depend on free-form strings for control decisions. Strings are for operator
messages; machine decisions use constants/typed fields.

### 6.2 `StartTask.srv`

```text
string task_id
bool request_auto_mode
bool dry_run
string request_id
---
bool accepted
uint8 reason_code
string message
string execution_id
```

This service returns after validation and queue/goal acceptance. It does not wait for
task completion.

### 6.3 `StopTask.srv`

```text
string execution_id
bool return_to_manual
---
bool accepted
string message
```

Require `execution_id` so a delayed client cannot cancel a newer task accidentally.
An explicit administrative force-cancel may be added later but must not be the normal
GUI path.

### 6.4 `MissionStatus.msg`

Include at least:

- mission state constant;
- task ID and execution ID;
- stage and progress;
- message;
- requested/effective control mode;
- AUTO permission;
- inhibit reasons;
- last transition timestamp.

Publish it transient-local so a newly connected GUI receives current state.

### 6.5 `ControlState.msg`

Include at least:

- requested mode: MANUAL or AUTO;
- effective source: ZERO, MANUAL, or AUTO;
- emergency-stop state;
- `auto_permitted`;
- `manual_command_fresh` and `nav_command_fresh`;
- inhibit reason codes and messages;
- timestamp.

### 6.6 Task discovery

`ListTasks.srv` returns `TaskInfo[]`. Each `TaskInfo` reports:

- task ID and human-readable name;
- availability (`AVAILABLE`, `EXPERIMENTAL`, `NOT_IMPLEMENTED`, `DISABLED`);
- reason/details;
- whether dry-run and automatic mode requests are supported.

The GUI must query this service instead of hard-coding task buttons.

## 7. Mission State Machine

Use one serialized event loop or lock-protected state machine. Do not let ROS action,
service, Nav2 result, timer, and health callbacks mutate mission state independently.

```text
IDLE
  -> VALIDATING
  -> CONFIGURING
  -> WAITING_FOR_READINESS
  -> WAITING_FOR_AUTO_PERMISSION
  -> RUNNING
  -> SUCCEEDED | FAILED | CANCELING
  -> IDLE
```

Required behavior:

- Reject unknown, disabled, or unimplemented tasks without side effects.
- Permit only one active execution.
- Treat repeated `request_id` as idempotent and return the existing execution.
- Reject a new task while another is active unless an explicit future replace API is
  introduced.
- Generate a unique `execution_id` for every accepted new execution.
- Ignore late callbacks from an old `execution_id`.
- Cancel active Nav2 goals before reporting task cancellation complete.
- Cancel/destroy Task 3 wait timers.
- On Mission Manager restart, start in `IDLE`; do not restore AUTO or resend a task
  automatically.
- On process shutdown, request Nav2 cancellation and allow Command Arbiter to fall
  back to zero.

Suggested mission state constants:

```text
IDLE
VALIDATING
CONFIGURING
WAITING_FOR_READINESS
WAITING_FOR_AUTO_PERMISSION
RUNNING
PAUSED
CANCELING
SUCCEEDED
FAILED
REJECTED
```

`PAUSED` should only be implemented after pause/resume semantics are specified. For
the first implementation, safety loss during `RUNNING` should command zero and abort
or remain inhibited according to an explicit policy with a timeout.

## 8. Control State and Safety Policy

### 8.1 Control state model

Keep mission state separate from control state:

```text
BOOTING
MANUAL
AUTO_REQUESTED
AUTO_ARMED
AUTO_RUNNING
AUTO_INHIBITED
EMERGENCY_STOP
```

`request_auto_mode: true` requests AUTO but does not itself grant permission.

### 8.2 Single safety decision owner

Safety Supervisor computes:

```text
auto_permitted =
    emergency_stop_clear
    and nav2_ready
    and task_ready
    and all enabled common requirements
    and all enabled task requirements
```

It publishes every failed condition as an inhibit reason. Command Arbiter consumes
the result and does not reimplement RTK, heartbeat, localization, or task policy.

### 8.3 Command Arbiter behavior

```text
if emergency_stop:
    publish zero
elif effective_source == MANUAL and manual_command_fresh:
    publish manual command
elif effective_source == AUTO and auto_permitted and nav_command_fresh:
    publish Nav2 command
else:
    publish zero
```

Continue publishing zero at the normal command rate when inhibited. Do not rely on a
downstream driver timeout as the primary stop mechanism.

### 8.4 Example policy

```yaml
control_policy:
  common:
    require_emergency_stop_clear: true
    require_nav2_ready: true
    require_task_ready: true
    require_fresh_nav_command: true
    nav_command_timeout_sec: 0.5

    require_ground_station: false
    require_driver_heartbeat: false
    require_localization_heartbeat: false
    require_rtk_fix: false
    require_no_critical_health_fault: false

  task1:
    require_waypoint_plan: true
    require_collision_monitor: false
    require_buoy_perception: false

  task2:
    require_waypoint_plan: true
    require_collision_monitor: true
    require_buoy_perception: true

  task3:
    require_waypoint_plan: true
    require_collision_monitor: true
    require_dynamic_gate_tf: true
```

Validate policy at startup. Reject unknown keys, unknown health signal names, and
type mismatches. Avoid comments as the mechanism for enabling/disabling requirements.

## 9. Task Registry

Create one registry that is both machine-readable and operator-visible.

```yaml
tasks:
  task1:
    display_name: Task 1
    availability: experimental
    executor: waypoint_sequence
    route: tasks/task1.yaml
    frame_id: odom
    nav2_profile: task1
    control_policy: task1
    features:
      collision_monitor: false
      cardinal_walls: true
      dynamic_gate_midpoints: false

  task1_skip_1_1:
    display_name: Task 1 GPS smoke test
    availability: experimental
    executor: waypoint_sequence
    route: tasks/task1.yaml
    route_variant: task1_skip_1_1
    frame_id: odom
    nav2_profile: task1
    control_policy: task1

  task1_2:
    display_name: Task 1-2 marker-driven route
    availability: not_implemented
    reason: Marker-driven route generation is not implemented

  task2:
    display_name: Task 2
    availability: experimental
    executor: waypoint_sequence
    route: tasks/task2.yaml
    frame_id: map
    nav2_profile: task2
    control_policy: task2
    features:
      collision_monitor: true

  task3_1:
    display_name: Task 3-1
    availability: experimental
    executor: staged_docking
    route: tasks/task3.yaml
    route_variant: task3_1
    frame_id: odom
    nav2_profile: task3
    control_policy: task3
    features:
      collision_monitor: true
      dynamic_gate_midpoints: true
      cardinal_walls: true

  task3_2:
    display_name: Task 3-2
    availability: experimental
    executor: staged_docking
    route: tasks/task3.yaml
    route_variant: task3_2
    frame_id: odom
    nav2_profile: task3
    control_policy: task3
```

Registry loading must verify that referenced executor names, route files, policy
names, and Nav2 profiles exist before reporting a task as runnable.

## 10. Waypoint Publisher Migration

Refactor `waypoint_publisher_node.py` into reusable pieces rather than adding more
startup parameters.

### 10.1 `WaypointConfigLoader`

- Load and validate task waypoint YAML.
- Convert selected route/variant to typed internal waypoints.
- Validate unique IDs, frame, finite coordinates, finite yaw, stage references, and
  non-empty required routes.
- Do not send Nav2 goals.

### 10.2 `WaypointSequenceExecutor`

- Execute Task 1 and Task 2 sequences.
- Build and publish the planned path transient-locally.
- Send `NavigateThroughPoses`.
- Wait for result for every task, not only Task 3.
- Convert Nav2 rejection/abort/cancel into typed mission results.
- Check execution ID in every callback.

### 10.3 `StagedDockingExecutor`

Move the existing Task 3 stage behavior here:

- gate stage;
- dock approach;
- berth wait;
- exit/next berth;
- final exit;
- retry policy.

Add reliable cancellation during:

- Nav2 goal acceptance;
- active navigation;
- retry scheduling;
- berth wait timer;
- stage transition.

### 10.4 Base executor contract

Every executor implements conceptually:

```python
async def validate(context) -> ValidationResult
async def prepare(context) -> PrepareResult
async def run(context, feedback_cb) -> ExecutionResult
async def cancel(execution_id) -> None
async def cleanup(context) -> None
```

Cleanup must be safe and idempotent after success, failure, cancellation, and partial
preparation.

## 11. Heartbeat and Health Refactor

### 11.1 Role heartbeat inventory

Replace duplicate-key/comment-driven YAML with one unambiguous list per role:

```yaml
signals:
  gnss:
    topic: /sensor/vehicle_gnss/fix/raw
    type: sensor_msgs/msg/NavSatFix
    timeout_sec: 1.0
    expected_frequency_hz: 20.0

  local_odometry:
    topic: /odometry/filtered/local
    type: nav_msgs/msg/Odometry
    timeout_sec: 1.0
    expected_frequency_hz: 30.0

  back_camera:
    topic: /back_cam/image_raw
    type: sensor_msgs/msg/Image
    timeout_sec: 1.0
    expected_frequency_hz: 30.0
```

An optional `enabled: false` may disable creation of a monitor, but it must not be
used to express whether the signal inhibits AUTO. That belongs to control policy.

### 11.2 Health output

Publish a `HealthState` containing named `HealthSignal` entries with:

- name;
- state (`UNKNOWN`, `OK`, `DEGRADED`, `ERROR`, `STALE`, `DISABLED`);
- age;
- measured frequency;
- message;
- timestamp.

Safety Supervisor matches policy requirements by signal name. It must reject a policy
that requires an absent signal.

### 11.3 Diagnostics and lamp

- Continue publishing ROS diagnostics for tools and logs.
- Do not use `any DiagnosticArray status >= ERROR` as an undifferentiated propulsion
  interlock.
- Alert Lamp consumes `ControlState`, `MissionStatus`, and selected health summary.
- Alert Lamp does not independently decide whether Command Arbiter may move.
- Keep lamp hardware driver watchdog behavior independent of policy evaluation.

## 12. Nav2 Task Profile Strategy

Current task launches use multiple Nav2 parameter files. Runtime switching requires a
deliberate migration.

### 12.1 Inventory first

Diff at least:

- `robot/config/nav2_params_humble.yaml`;
- `robot/config/nav2_params_task2_humble.yaml`;
- `robot/config/nav2_params_task3_humble.yaml`;
- Jazzy equivalents if they remain supported.

Classify every difference as:

1. common static configuration;
2. dynamically settable parameter;
3. lifecycle reconfiguration requirement;
4. plugin topology difference that cannot be changed safely at runtime;
5. obsolete or accidental divergence.

### 12.2 Target

- Put static common configuration in one base file.
- Start a superset of required plugins where practical.
- Put task-selectable values in named profile overlays.
- Apply overlays only while effective output is ZERO/MANUAL.
- Deactivate affected Nav2 lifecycle nodes.
- Apply parameters atomically.
- Reactivate and verify action servers, TF, costmaps, and required topics.
- Roll back on failure and keep AUTO inhibited.

### 12.3 Initial limitation is acceptable

If full runtime profile switching cannot be made safe in the first increment, allow
only tasks compatible with the active Nav2 base profile and reject incompatible tasks
with a specific message. Do not hide the limitation or partially apply a profile.

## 13. Role Bringup Changes

### 13.1 miniPC

Start persistently:

- hardware owned by miniPC;
- localization;
- thruster driver;
- Control Manager and Command Arbiter;
- Health/Safety Supervisor;
- Mission Manager;
- Nav2 common graph;
- alert lamp;
- networking.

Do not automatically submit a task goal. Startup must end in MANUAL, mission `IDLE`,
and zero autonomous output.

### 13.2 Jetson

Start persistently:

- LiDAR, ZED, GLIM according to launch arguments;
- task-support perception components;
- Jetson health monitors;
- networking.

Heavy optional components should eventually become lifecycle-managed or expose a
typed enable/disable control. Do not dynamically launch them from Mission Manager.

### 13.3 Ground PC

Start persistently:

- joystick and critical-link sender;
- ground-station heartbeat;
- video receivers;
- Zenoh networking;
- GUI bridge/task client.

`ground_pc.launch.py` currently defines but comments out the Foxglove bridge launch.
Reintroduce it behind an explicit `enable_foxglove_bridge` launch argument when the
GUI service path is implemented.

### 13.4 Legacy task launch compatibility

During migration, convert `task1.launch.py`, `task2.launch.py`, and `task3.launch.py`
into deprecated wrappers or test-only compatibility launch files. They must not start
duplicate hardware after role bringup is the normal workflow.

Suggested transition:

1. Keep existing launch behavior for regression tests.
2. Add explicit deprecation warnings and a new `start_role_bringup` argument.
3. Add a wrapper path that calls the Mission API after readiness.
4. Make role bringup + Mission API the documented default.
5. Remove legacy automatic goal submission only after equivalent integration tests
   pass.

## 14. GUI and CLI

### 14.1 CLI commands

Provide a small package entry point, for example:

```bash
njord-task list
njord-task status
njord-task check task1
njord-task start task1 --auto
njord-task start task3_1 --dry-run
njord-task stop
njord-task manual
```

`check` performs the same validation/preparation checks without sending a Nav2 goal
or switching effective control mode.

### 14.2 GUI fields

Display:

- task list and availability;
- selected/current task;
- mission state and stage;
- progress;
- requested and effective mode;
- AUTO permission;
- inhibit reasons;
- emergency-stop state;
- Nav2 readiness;
- planned waypoint count;
- named health signals;
- last state transition and error.

### 14.3 Safe start flow

Recommended GUI flow:

```text
select task
  -> dry-run/check
  -> display readiness and inhibit reasons
  -> operator confirmation
  -> StartTask(request_auto_mode=true)
```

Task start is still subject to emergency stop and all enabled safety requirements.

## 15. Communication and Ownership

- Mission Manager on miniPC is the single canonical task owner.
- Do not use an untyped string Topic for start commands.
- Use request IDs and execution IDs for idempotency and stale-request protection.
- Initially transport Action/Service through the ROS/Zenoh graph.
- Maintain the existing rule that Ground PC joystick data cannot directly contend
  with vessel canonical control Topics.
- Consider extending `critical_link` later with an acknowledged mission-command
  packet if competition reliability requires it.
- A lost GUI connection does not automatically cancel a task unless
  `require_ground_station` is enabled in policy.
- No remote request may bypass E-stop or directly publish `/cmd_vel`.

## 16. Implementation Phases

Each phase should be a small reviewable local commit or series of commits. Repository
rules require local commits for implementation work. Do not mix unrelated existing
worktree changes.

### Phase 0: Baseline and specifications

Tasks:

- Record current node/topic/action ownership for each role.
- Produce a table of nodes started by every task and role launch.
- Diff Task 1/2/3 Nav2 parameter files.
- Record current command arbitration truth table in tests.
- Record current waypoint sequences and Task 3 stage behavior in tests.
- Classify every known task as available, experimental, not implemented, or disabled.
- Correct misleading comments only in a dedicated commit.

Acceptance:

- Tests fail if existing command gating or waypoint generation changes unexpectedly.
- No implementation task begins without a documented current owner and replacement
  owner.

### Phase 1: Typed interfaces

Tasks:

- Add the interfaces in Section 6 to `njord_interfaces`.
- Update CMake/package dependencies.
- Add interface generation tests or build verification.
- Define stable constants and result codes.

Acceptance:

- Interfaces build on supported ROS distributions.
- A dummy Action server/client can send goal, feedback, result, and cancellation.
- GUI service requests can represent all supported task IDs without free-form
  parameter/value configuration.

### Phase 2: Mission Manager skeleton

Tasks:

- Implement registry loader and validation.
- Implement serialized mission state machine.
- Implement `RunTask`, Start/Stop/List/GetStatus APIs.
- Implement request-id idempotency and execution IDs.
- Implement a dummy executor.
- Publish transient-local MissionStatus.

Acceptance:

- Unknown/not-implemented/disabled tasks are rejected without side effects.
- Only one execution is accepted.
- Duplicate request ID returns the existing execution.
- Restart returns to IDLE without starting a task.
- Cancel transitions deterministically to IDLE.

### Phase 3: Existing waypoint behavior as executors

Tasks:

- Extract config loading.
- Implement waypoint sequence executor for Task 1/2.
- Implement staged docking executor for Task 3.
- Publish planned routes.
- Wait for Nav2 result for all tasks.
- Add cancellation and stale-callback protection.
- Keep current waypoint files unchanged until equivalence tests pass.

Acceptance:

- Generated pose sequences match baseline tests.
- Goal rejection, abort, success, and cancel map to correct mission results.
- Cancel leaves no active Nav2 task goal.
- Task 3 can be canceled during every stage and wait timer.

### Phase 4: Persistent Mission Manager in role bringup

Tasks:

- Add Mission Manager to miniPC bringup.
- Ensure role bringup never sends an automatic task goal.
- Move Nav2 toward persistent common ownership.
- Keep legacy task launches available for comparison.
- Add launch tests for single ownership.

Acceptance:

- `minipc_bringup` reaches MANUAL/IDLE.
- Task can be started later through CLI without another hardware bringup.
- Mission Manager, Nav2, control, and hardware owners exist at most once.

### Phase 5: Control Manager and Safety Supervisor

Tasks:

- Move mode state out of JoyConverter; publish typed mode requests.
- Implement requested/effective mode separation.
- Implement policy loading and inhibit reasons.
- Simplify Command Arbiter to the logic in Section 8.
- Preserve emergency-stop priority.
- Publish ControlState transient-locally.
- Resolve overlapping joystick button assignments in a separately reviewed config
  change.

Acceptance:

- Every zero-output decision has an observable reason.
- AUTO request never bypasses E-stop or policy.
- Stale Nav2 command produces zero within the configured timeout.
- MANUAL remains usable while no task is selected.
- Restart defaults to MANUAL and zero autonomous output.

### Phase 6: Health and heartbeat consolidation

Tasks:

- Replace `minipc_heartbeat.yaml` duplicate-key structure.
- Move embedded role/topic inventories into validated role YAML.
- Publish typed HealthState.
- Connect named health requirements to Safety Supervisor.
- Remove undifferentiated global diagnostic ERROR gating.
- Make Alert Lamp a consumer of canonical state.

Acceptance:

- Every monitored signal appears in status even if its control requirement is OFF.
- Policy ON/OFF is visible from one config file.
- Requiring an unknown signal fails configuration safely.
- An irrelevant diagnostic ERROR cannot inhibit propulsion.
- A required stale signal inhibits AUTO with a named reason.

### Phase 7: Nav2 profile consolidation

Tasks:

- Complete the parameter inventory.
- Create common base and typed task overlays.
- Implement safe lifecycle/profile apply with rollback.
- Verify collision-monitor state rather than assuming a parameter call succeeded.
- Block Task start until Nav2 action servers and required outputs are ready.

Acceptance:

- Supported task switches do not restart hardware bringup.
- Profile changes occur only with effective command output at zero.
- Partial apply keeps AUTO inhibited and reports configuration failure.
- Task 1, Task 2, and Task 3 profiles are distinguishable in status.

### Phase 8: CLI and GUI

Tasks:

- Add `njord-task` CLI.
- Enable Foxglove bridge behind a launch argument.
- Add task list/status/start/stop GUI controls.
- Display inhibit and health details.
- Add operator confirmation and dry-run flow.

Acceptance:

- GUI and CLI use the same canonical APIs.
- Reconnecting GUI shows current transient-local state.
- Double-click/repeated requests do not create duplicate executions.
- Stop targets the current execution ID and cancels Nav2.

### Phase 9: Remove obsolete paths

Tasks:

- Deprecate/remove automatic task goal submission from task launch files.
- Move control logic out of `simple_manual` after replacement is proven.
- Decide whether to remove or narrowly retain ConfigureSystem/system_manager.
- Remove unused task Behavior Trees only after confirming no launch references.
- Update operator docs and diagrams.

Acceptance:

- One documented workflow exists for real operation.
- No duplicate mode owner, safety decision owner, or command arbiter remains.
- Legacy files have either a documented compatibility purpose or are removed.

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

## 18. Recommended First Implementation Slice

Keep the first implementation narrow:

1. Add Mission interfaces.
2. Add validated task registry.
3. Add Mission Manager with dummy executor.
4. Convert existing Task 1 waypoint path into an executor.
5. Start Mission Manager from miniPC bringup without changing current control gating.
6. Add CLI list/check/start/stop.
7. Prove no automatic goal is sent at bringup.

The first milestone is:

```bash
ros2 launch robot minipc_bringup.launch.py
njord-task list
njord-task check task1
njord-task start task1 --auto
njord-task stop
```

At this milestone, document that control/safety and heartbeat consolidation are still
pending. Do not claim the full architecture is complete until Phases 5 and 6 pass.

## 19. Implementation Guardrails

- Preserve unrelated worktree changes; stage explicit paths only.
- Make a local commit for every implementation task as required by repository rules.
- Do not push or create a PR unless explicitly requested and repository-owner checks
  pass.
- Never change hardware ownership and mission behavior in one unreviewable commit.
- Default to MANUAL, mission IDLE, and zero autonomous output after restart.
- Do not auto-resume a mission after process or machine restart.
- Do not use YAML comments as runtime feature flags.
- Validate every YAML file and reject duplicate keys where possible.
- Do not use arbitrary string parameter/value APIs for safety-critical transitions.
- Never report a task READY until all required configuration has been applied and
  verified.
- Keep Task selection, operator mode request, safety permission, and effective command
  source separately observable.

## 20. Definition of Done

The refactor is complete only when all of the following are true:

- Each machine is started through its role bringup once.
- A task can be listed, checked, started, monitored, canceled, and changed through a
  typed runtime API.
- Task switching does not duplicate or relaunch hardware ownership.
- Unsupported tasks are rejected with a clear reason.
- AUTO requested, AUTO permitted, and AUTO running are distinct states.
- All active AUTO requirements are defined in one validated policy.
- Health conditions remain observable when their policy requirement is OFF.
- Command Arbiter has one small, test-covered selection policy.
- Alert Lamp displays canonical control/mission/health state rather than making a
  competing propulsion decision.
- Nav2 goal cancellation and safety faults reliably produce zero command.
- GUI and CLI use the same Mission API.
- Legacy task launch paths are removed or explicitly documented as compatibility
  tools.
- Automated unit, integration, and launch tests cover safety loss, cancellation,
  duplicate requests, restarts, and task transitions.
