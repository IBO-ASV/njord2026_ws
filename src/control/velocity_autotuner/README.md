# Velocity autotuner (surge)

This ROS 2 node tunes surge feed-forward, P, then D gains with bounded trials. It is opt-in and does not start vessel motion by itself. Start only after the vessel is in a suitable test area and the ordinary control state is ready in AUTO.

## Run

Build `velocity_autotuner`, `thruster_controller`, and `motion_supervisor`, then launch the node with explicit values. There are intentionally no motion or thrust defaults:

```sh
scripts/operations/run_agx_velocity_tuner.sh
```

This helper selects the AGX ROS domain and network configuration before launching the idle tuner. The equivalent direct launch is `ros2 launch velocity_autotuner velocity_autotuner.launch.py` after sourcing the same AGX environment.

This starts the tuner idle. Set target speed, speed limit, and thrust limit in the Mission Control panel, then press start. For command-line use, the `ros2 run` entry point also accepts these values as parameters.

The example values are placeholders; choose test limits for the specific vessel and site. The tuner temporarily caps each controller thruster output at the smaller of the requested cap and configured thruster maximum, and sets surge I gain to zero. It restores the saved cap and I gain when stopped. A completed run keeps the selected FF/P/D gains; an incomplete run restores the previous gains.

Services (all `std_srvs/Trigger`):

- `/velocity_tuning/start`: begin after fresh base-link odometry and a ready AUTO state are present.
- `/velocity_tuning/pause`: immediately publish zero and hold the current candidate. Select MANUAL using the normal mode interface to take over and reposition.
- `/velocity_tuning/resume`: after repositioning, return to ready AUTO, then continue the interrupted candidate from a fresh trial.
- `/velocity_tuning/stop`: select MANUAL, then stop and restore the temporary settings.

Emergency stop, manual/SBUS selection, stale odometry or thrust feedback, and exceeding the configured speed limit pause the tuner and produce a zero tuning command. The command arbiter still gives emergency stop and manual/SBUS their normal priority. Do not bypass the normal safety gate.

Results are written under `output_dir` (default `~/njord_tuning`): a CSV of trial metrics and a YAML with the selected gains. FF uses steady-state speed error; P and D use integrated absolute error, overshoot, and thrust slew. Search is bounded interval refinement, not an assumption that every response is monotonic.

This version tunes surge velocity gains only. Matching the individual thruster output ratios (FR/FL/RR/RL) remains a separate, manually measured calibration; the tuner does not issue direct per-thruster movement commands.

The Mission Control Foxglove panel sets the three test limits before starting through `/velocity_autotuner/set_parameters_atomically`; the operator does not need to restart the tuner to change them. Parameters are locked while a session is reserved.
