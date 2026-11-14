# System runtime services

- `runtime_manager` owns the selected Nav2 runtime process and profile. It serves
  `/system/configure` using `runtime_interfaces/ConfigureSystem` and publishes
  `/runtime/nav2/status`. Mission execution requests a profile through this API.
- `parameter_gateway` provides the legacy `configure_system` action for remote
  parameter updates. It is an optional adapter, not the Nav2 process owner.

Mission state machines live in `src/mission/mission_manager`; operator command
validation and transport adapters live in `src/operator/operator_gateway`.
`src/bringup/robot/launch/mission_stack.launch.py` composes these services.
