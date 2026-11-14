# Velocity autotuner (surge)

This ROS 2 node tunes surge feed-forward, P, then D gains with bounded trials. It is opt-in and does not start vessel motion by itself. Start only after the vessel is in a suitable test area and the ordinary control state is ready in AUTO.

## Run

Build `velocity_autotuner`, `thruster_controller`, and `motion_supervisor`, then launch the node with explicit values. There are intentionally no motion or thrust defaults:

```sh
ros2 run velocity_autotuner velocity_autotuner --ros-args \
  -p target_speed_mps:=0.2 -p max_test_speed_mps:=0.4 \
  -p test_max_thruster_thrust_n:=10.0
```

The example values are placeholders; choose test limits for the specific vessel and site. The tuner temporarily caps each controller thruster output at the smaller of the requested cap and configured thruster maximum, and sets surge I gain to zero. It restores the saved cap and I gain when stopped. A completed run keeps the selected FF/P/D gains; an incomplete run restores the previous gains.

Services (all `std_srvs/Trigger`):

- `/velocity_tuning/start`: begin after fresh base-link odometry and a ready AUTO state are present.
- `/velocity_tuning/pause`: immediately publish zero and hold the current candidate. Select MANUAL using the normal mode interface to take over and reposition.
- `/velocity_tuning/resume`: after repositioning, return to ready AUTO, then continue the interrupted candidate from a fresh trial.
- `/velocity_tuning/stop`: select MANUAL, then stop and restore the temporary settings.

Emergency stop, manual/SBUS selection, stale odometry or thrust feedback, and exceeding the configured speed limit pause the tuner and produce a zero tuning command. The command arbiter still gives emergency stop and manual/SBUS their normal priority. Do not bypass the normal safety gate.

Results are written under `output_dir` (default `~/njord_tuning`): a CSV of trial metrics and a YAML with the selected gains. FF uses steady-state speed error; P and D use integrated absolute error, overshoot, and thrust slew. Search is bounded interval refinement, not an assumption that every response is monotonic.

This version tunes surge velocity gains only. Matching the individual thruster output ratios (FR/FL/RR/RL) remains a separate, manually measured calibration; the tuner does not issue direct per-thruster movement commands.
