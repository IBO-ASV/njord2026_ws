# Task2 normal-shell startup repair (2026-11-13)

The user's `ros2 launch njord_sim mission_sim.launch.py task:=task2 noise:=true`
started Nav2 successfully: controller, planner and navigator lifecycle states
were ACTIVE. However MPPI `planner_node` exited with code 1: normal ROS Python
could not import `torch`. `/planned_path_pruned` had zero publishers and
`/autonomy/ready` was false. The opponent-selector no-track warning was not the
startup cause. Earlier acceptance runs had used an explicitly exported temporary
Python dependency directory, which was missing from the interactive shell.

Copied the already available CPU environment using
`scripts/install_task2_dependencies.py --from-directory /run/user/1000/njord-sim-deps`
to the persistent Python user site. PyTorch 2.15.0+cpu now imports from
`/home/ibo/.local/lib/python3.12/site-packages/torch`, without adding that temporary
directory to PYTHONPATH. On this host apt has no python3-torch candidate and sudo
requires a password, so no system package changes were made.
Task2 launch now performs an actual PyTorch CPU operation before starting nodes;
missing/broken dependencies produce an actionable startup error rather than a
partially running simulation. Other task launches do not need this check.

No controller or MPPI parameter was changed. Hardware drivers and real sensors
were not started. The user's existing launch on domain 89 was left running.
The isolated run below uses the same Task2/noise setup, with the GUI bridge off
so it does not bind the user's existing websocket port:

```
python3 scripts/run_mission_sim.py --task task2 --noise --domain 126 \
  --timeout 300 --output /run/user/1000/njord-task2-torch-fix-20261113
```

Result: passed, Mission SUCCEEDED, both gates passed, opponent selected, minimum
buoy hull clearance 0.816 m, minimum opponent hull clearance 2.945 m,
final command zero and stationary for 3.299 s. Goal reached about 39.4 s after
AUTO execution started. The manifest records the pre-commit dirty working tree.
The headless runner's process-group cleanup can deliver repeated SIGINT during
teardown; post-evaluation KeyboardInterrupt logs are not planner startup faults.

Retained evidence: `run.json`, `evaluation.json`, `observer.log`.
Regression checks: 76 control/mission/simulation tests passed, including missing,
healthy and timed-out PyTorch startup probes. No tuning changes were needed.
