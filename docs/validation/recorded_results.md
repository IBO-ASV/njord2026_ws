# 保存した検証条件と結果


詳細生成物は整理前 commit `74fe7ec6` に保存されています。必要時は `git show 74fe7ec6:<当時のパス>` で取得してください。以下は当時の結果で、今回の再実行結果ではありません。新規出力は `artifacts/` を使用します。


## Simulation acceptance


| Case | Group | Passed | Reason |
|---|---|---|---|

| cancel | acceptance | True | mission terminal |

| sensor-loss | acceptance | True | mission terminal |

| task1-noise2026 | acceptance | True | mission terminal |

| task1-noise2027 | acceptance | True | mission terminal |

| task1-noise2028 | acceptance | True | mission terminal |

| task1-truth | acceptance | True | mission terminal |

| task1_2-mmg | acceptance | True | mission terminal |

| task1_2-noise2026 | acceptance | True | mission terminal |

| task1_2-noise2027 | acceptance | True | mission terminal |

| task1_2-noise2028 | acceptance | True | mission terminal |

| task1_2-truth | acceptance | True | mission terminal |

| task2-head_on-noise2026 | acceptance | True | mission terminal |

| task2-head_on-noise2027 | acceptance | True | mission terminal |

| task2-head_on-noise2028 | acceptance | True | mission terminal |

| task2-head_on-truth | acceptance | True | mission terminal |

| task2-none-noise2026 | acceptance | True | mission terminal |

| task2-none-noise2027 | acceptance | True | mission terminal |

| task2-none-noise2028 | acceptance | True | mission terminal |

| task2-none-truth | acceptance | True | mission terminal |

| task2-port-noise2026 | acceptance | True | mission terminal |

| task2-port-noise2027 | acceptance | True | mission terminal |

| task2-port-noise2028 | acceptance | True | mission terminal |

| task2-port-truth | acceptance | True | mission terminal |

| task2-starboard-noise2026 | acceptance | True | mission terminal |

| task2-starboard-noise2027 | acceptance | True | mission terminal |

| task2-starboard-noise2028 | acceptance | True | mission terminal |

| task2-starboard-truth | acceptance | True | mission terminal |

| task3_1-noise2026 | acceptance | True | mission terminal |

| task3_1-noise2027 | acceptance | True | mission terminal |

| task3_1-noise2028 | acceptance | True | mission terminal |

| task3_1-truth | acceptance | True | mission terminal |

| task3_2-noise2026 | acceptance | True | mission terminal |

| task3_2-noise2027 | acceptance | True | mission terminal |

| task3_2-noise2028 | acceptance | True | mission terminal |

| task3_2-truth | acceptance | True | mission terminal |

| task4-noise2026 | acceptance | True | mission terminal |

| task4-noise2027 | acceptance | True | mission terminal |

| task4-noise2028 | acceptance | True | mission terminal |

| task4-truth | acceptance | True | mission terminal |

| task2-headon-oscillating-guard-noise2028 | superseded_failure | False | mission terminal |

| task2-port-mppi-only | superseded_failure | False | mission terminal |


以下に引用する旧READMEの「保持」やローカル保存先は検証当時の記録です。詳細ログ・画像・計測JSONの現在の取得先は上記Git履歴です。

## 実行条件


下記のrun manifestからCLI引数、seed・model・encounter・commitを保存しています。dirty treeの試行はcommitだけでは完全再現できません。生成された設定は `fixtures/` に保持しています。詳細なsource hashとworking-tree状態は上記Git履歴の元manifestを参照してください。



### cpu_runtime.json


```json
{
  "time": "2026-11-13T09:17:48.985290+00:00",
  "python": "/home/ibo/njord2026_ws/.venv/bin/python",
  "platform": "aarch64",
  "requested_device": "cpu",
  "passed": true,
  "versions": {
    "numpy": "1.26.4",
    "scipy": "1.15.3",
    "opencv-python": "4.10.0.84",
    "matplotlib": "3.10.8",
    "polars": "1.36.1",
    "torch": "2.9.1+cpu",
    "torchvision": "0.24.1",
    "ultralytics": "8.3.237"
  },
  "cuda_build": null,
  "cuda_available": false,
  "numeric_image_plot_checks": true,
  "tensor_and_nms_checks": true,
  "ros_cv_bridge_roundtrip": true,
  "rclpy": "/opt/ros/jazzy/lib/python3.12/site-packages/rclpy/__init__.py",
  "yolo": {
    "model": "/home/ibo/njord2026_ws/src/robot/config/yolo_model/best.pt",
    "device": "cpu",
    "classes": {
      "0": "green",
      "1": "red",
      "2": "north",
      "3": "east",
      "4": "south",
      "5": "west"
    },
    "detections": 0,
    "timing_ms": [
      294.4576719892211,
      128.35232500219718,
      132.4552349979058
    ],
    "input": "synthetic black 320x320 BGR; not an accuracy test"
  }
}
```


### jetson_runtime.json


```json
{
  "time": "2026-11-13T09:18:53.831508+00:00",
  "python": "/home/ibo/njord2026_ws/.venv/bin/python",
  "platform": "aarch64",
  "requested_device": "cuda:0",
  "passed": true,
  "versions": {
    "numpy": "1.26.4",
    "scipy": "1.15.3",
    "opencv-python": "4.10.0.84",
    "matplotlib": "3.10.8",
    "polars": "1.36.1",
    "torch": "2.9.1+cu130",
    "torchvision": "0.24.1",
    "ultralytics": "8.3.237"
  },
  "cuda_build": "13.0",
  "cuda_available": true,
  "gpu": "Orin",
  "compute_capability": [
    8,
    7
  ],
  "numeric_image_plot_checks": true,
  "tensor_and_nms_checks": true,
  "triton_kernel": true,
  "ros_cv_bridge_roundtrip": true,
  "rclpy": "/opt/ros/jazzy/lib/python3.12/site-packages/rclpy/__init__.py",
  "yolo": {
    "model": "/home/ibo/njord2026_ws/src/robot/config/yolo_model/best.pt",
    "device": "cuda:0",
    "classes": {
      "0": "green",
      "1": "red",
      "2": "north",
      "3": "east",
      "4": "south",
      "5": "west"
    },
    "detections": 0,
    "timing_ms": [
      738.8779330067337,
      25.81728796940297,
      24.789720016997308
    ],
    "input": "synthetic black 320x320 BGR; not an accuracy test"
  }
}
```


### ros_yolo_pipeline.json


```json
{
  "passed": true,
  "device": "cuda:0",
  "input": "synthetic 320x320 BGR ROS Image",
  "output_topic": "/runtime_check/detections",
  "output_messages": 1,
  "detections": 0,
  "sensors_used": false
}
```


### ros_yolo_positive_pipeline.json


```json
{
  "passed": true,
  "device": "cuda:0",
  "input": "synthetic 320x320 noise + red/green circles",
  "output_topic": "/runtime_check/detections",
  "output_messages": 1,
  "detections": 5,
  "sensors_used": false,
  "confidence_threshold": 1e-06,
  "purpose": "force positive message serialization, not detection accuracy; runtime defaults unchanged"
}
```


### simulation_results/README.md


# Retained simulation evidence

`acceptance.json` is the case index, including metrics, the raw run directory,
and SHA256 digests of generated configuration. `group: acceptance` contains
completed selected tests; `superseded_failure` contains two representative
failed tuning attempts. It is not a count of every trial attempted.

Every selected case retains its original `evaluation.json` and `run.json`.
Representative cases additionally retain generated config, truth CSV and PNG.
CSV line endings and trailing whitespace in retained text logs are normalized;
raw files remain in the run directories. The manifest records the seed, model, encounter, command arguments, commit,
working-tree status and Python source hashes. A commit alone does not identify
a dirty trial's exact configuration.

The final compiled goal-checker source/library hashes and runtime versions are
in `build_fingerprints.json`.

Full stdout/stderr, intermediate successes/failures and interrupted overload
trials remain in `/tmp/njord-night/` on the test host. They are not required to
view these retained summaries and representative trajectories.

`unit_tests.log`, `gui_tests.log`, `goal_checker_test.log` and
`foxglove_bridge_smoke.json` record focused verification. The separate
`baseline_robot_test_failures.log` preserves seven existing launch/network/video
assertion failures instead of treating the whole repository as green.

See [the Japanese report](simulation_acceptance.md) and
[tuning notes](simulation_tuning_log.md) for interpretation and limitations.



### simulation_results/cases/cancel/run.json

`{"task": "task1", "noise": false, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 65, "domain": 94, "encounter": "head_on", "cancel_after": 25.0, "dropout_after": 0.0, "commit": "1f19649fabf823e22441b3652d0bf4cb876ddac3"}`

### simulation_results/cases/sensor-loss/run.json

`{"task": "task1", "noise": false, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 65, "domain": 94, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 25.0, "commit": "1f19649fabf823e22441b3652d0bf4cb876ddac3"}`

### simulation_results/cases/task1-noise2026/run.json

`{"task": "task1", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 900, "domain": 125, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "cc7b04b25a6cb3e2630020cedf3e9975eff61a07"}`

### simulation_results/cases/task1-noise2027/run.json

`{"task": "task1", "noise": true, "disturbance": false, "seed": 2027, "model": "kinematic", "timeout": 900, "domain": 126, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "cc7b04b25a6cb3e2630020cedf3e9975eff61a07"}`

### simulation_results/cases/task1-noise2028/run.json

`{"task": "task1", "noise": true, "disturbance": false, "seed": 2028, "model": "kinematic", "timeout": 900, "domain": 126, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "cc7b04b25a6cb3e2630020cedf3e9975eff61a07"}`

### simulation_results/cases/task1-truth/run.json

`{"task": "task1", "noise": false, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 600, "domain": 126, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "bab873b8cde5ad4c2231347e8e9cf2df955f75cc"}`

### simulation_results/cases/task1_2-mmg/run.json

`{"task": "task1_2", "noise": false, "disturbance": false, "seed": 2026, "model": "mmg", "timeout": 300, "domain": 89, "encounter": "head_on", "cancel_after": 0.0, "commit": "1f19649fabf823e22441b3652d0bf4cb876ddac3"}`

### simulation_results/cases/task1_2-noise2026/run.json

`{"task": "task1_2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 900, "domain": 125, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "cc7b04b25a6cb3e2630020cedf3e9975eff61a07"}`

### simulation_results/cases/task1_2-noise2027/run.json

`{"task": "task1_2", "noise": true, "disturbance": false, "seed": 2027, "model": "kinematic", "timeout": 900, "domain": 125, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "cc7b04b25a6cb3e2630020cedf3e9975eff61a07"}`

### simulation_results/cases/task1_2-noise2028/run.json

`{"task": "task1_2", "noise": true, "disturbance": false, "seed": 2028, "model": "kinematic", "timeout": 900, "domain": 126, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "cc7b04b25a6cb3e2630020cedf3e9975eff61a07"}`

### simulation_results/cases/task1_2-truth/run.json

`{"task": "task1_2", "noise": false, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 900, "domain": 127, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "bab873b8cde5ad4c2231347e8e9cf2df955f75cc"}`

### simulation_results/cases/task2-head_on-noise2026/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 180, "domain": 119, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "a20ad49fb852410d20d94ce168259baa602d185e"}`

### simulation_results/cases/task2-head_on-noise2027/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2027, "model": "kinematic", "timeout": 180, "domain": 119, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "bab873b8cde5ad4c2231347e8e9cf2df955f75cc"}`

### simulation_results/cases/task2-head_on-noise2028/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2028, "model": "kinematic", "timeout": 180, "domain": 119, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "a20ad49fb852410d20d94ce168259baa602d185e"}`

### simulation_results/cases/task2-head_on-truth/run.json

`{"task": "task2", "noise": false, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 180, "domain": 119, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "a20ad49fb852410d20d94ce168259baa602d185e"}`

### simulation_results/cases/task2-headon-oscillating-guard-noise2028/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2028, "model": "kinematic", "timeout": 180, "domain": 119, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3106933ba344cf18c34d9ae058bcf570695c577f"}`

### simulation_results/cases/task2-none-noise2026/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 180, "domain": 119, "encounter": "none", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "bab873b8cde5ad4c2231347e8e9cf2df955f75cc"}`

### simulation_results/cases/task2-none-noise2027/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2027, "model": "kinematic", "timeout": 180, "domain": 119, "encounter": "none", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "bab873b8cde5ad4c2231347e8e9cf2df955f75cc"}`

### simulation_results/cases/task2-none-noise2028/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2028, "model": "kinematic", "timeout": 180, "domain": 119, "encounter": "none", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "bab873b8cde5ad4c2231347e8e9cf2df955f75cc"}`

### simulation_results/cases/task2-none-truth/run.json

`{"task": "task2", "noise": false, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 180, "domain": 119, "encounter": "none", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "bab873b8cde5ad4c2231347e8e9cf2df955f75cc"}`

### simulation_results/cases/task2-port-mppi-only/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 180, "domain": 119, "encounter": "port", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3106933ba344cf18c34d9ae058bcf570695c577f"}`

### simulation_results/cases/task2-port-noise2026/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 180, "domain": 119, "encounter": "port", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "bab873b8cde5ad4c2231347e8e9cf2df955f75cc"}`

### simulation_results/cases/task2-port-noise2027/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2027, "model": "kinematic", "timeout": 180, "domain": 119, "encounter": "port", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "bab873b8cde5ad4c2231347e8e9cf2df955f75cc"}`

### simulation_results/cases/task2-port-noise2028/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2028, "model": "kinematic", "timeout": 180, "domain": 119, "encounter": "port", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "bab873b8cde5ad4c2231347e8e9cf2df955f75cc"}`

### simulation_results/cases/task2-port-truth/run.json

`{"task": "task2", "noise": false, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 180, "domain": 119, "encounter": "port", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "bab873b8cde5ad4c2231347e8e9cf2df955f75cc"}`

### simulation_results/cases/task2-starboard-noise2026/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 180, "domain": 119, "encounter": "starboard", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "bab873b8cde5ad4c2231347e8e9cf2df955f75cc"}`

### simulation_results/cases/task2-starboard-noise2027/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2027, "model": "kinematic", "timeout": 180, "domain": 119, "encounter": "starboard", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "bab873b8cde5ad4c2231347e8e9cf2df955f75cc"}`

### simulation_results/cases/task2-starboard-noise2028/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2028, "model": "kinematic", "timeout": 180, "domain": 119, "encounter": "starboard", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "bab873b8cde5ad4c2231347e8e9cf2df955f75cc"}`

### simulation_results/cases/task2-starboard-truth/run.json

`{"task": "task2", "noise": false, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 180, "domain": 119, "encounter": "starboard", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "bab873b8cde5ad4c2231347e8e9cf2df955f75cc"}`

### simulation_results/cases/task3_1-noise2026/run.json

`{"task": "task3_1", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 900, "domain": 127, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3106933ba344cf18c34d9ae058bcf570695c577f"}`

### simulation_results/cases/task3_1-noise2027/run.json

`{"task": "task3_1", "noise": true, "disturbance": false, "seed": 2027, "model": "kinematic", "timeout": 900, "domain": 127, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3106933ba344cf18c34d9ae058bcf570695c577f"}`

### simulation_results/cases/task3_1-noise2028/run.json

`{"task": "task3_1", "noise": true, "disturbance": false, "seed": 2028, "model": "kinematic", "timeout": 900, "domain": 127, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3106933ba344cf18c34d9ae058bcf570695c577f"}`

### simulation_results/cases/task3_1-truth/run.json

`{"task": "task3_1", "noise": false, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 900, "domain": 127, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3106933ba344cf18c34d9ae058bcf570695c577f"}`

### simulation_results/cases/task3_2-noise2026/run.json

`{"task": "task3_2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 900, "domain": 127, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3106933ba344cf18c34d9ae058bcf570695c577f"}`

### simulation_results/cases/task3_2-noise2027/run.json

`{"task": "task3_2", "noise": true, "disturbance": false, "seed": 2027, "model": "kinematic", "timeout": 900, "domain": 127, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3106933ba344cf18c34d9ae058bcf570695c577f"}`

### simulation_results/cases/task3_2-noise2028/run.json

`{"task": "task3_2", "noise": true, "disturbance": false, "seed": 2028, "model": "kinematic", "timeout": 900, "domain": 127, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3106933ba344cf18c34d9ae058bcf570695c577f"}`

### simulation_results/cases/task3_2-truth/run.json

`{"task": "task3_2", "noise": false, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 900, "domain": 127, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3106933ba344cf18c34d9ae058bcf570695c577f"}`

### simulation_results/cases/task4-noise2026/run.json

`{"task": "task4", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 1800, "domain": 127, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "a20ad49fb852410d20d94ce168259baa602d185e"}`

### simulation_results/cases/task4-noise2027/run.json

`{"task": "task4", "noise": true, "disturbance": false, "seed": 2027, "model": "kinematic", "timeout": 1800, "domain": 127, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "bab873b8cde5ad4c2231347e8e9cf2df955f75cc"}`

### simulation_results/cases/task4-noise2028/run.json

`{"task": "task4", "noise": true, "disturbance": false, "seed": 2028, "model": "kinematic", "timeout": 1800, "domain": 127, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "bab873b8cde5ad4c2231347e8e9cf2df955f75cc"}`

### simulation_results/cases/task4-truth/run.json

`{"task": "task4", "noise": false, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 1800, "domain": 127, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3106933ba344cf18c34d9ae058bcf570695c577f"}`

### simulation_results/task2_persistent_torch_20261113/README.md


# Task2 normal-shell startup repair (2026-11-13)

The user's `ros2 launch njord_sim mission_sim.launch.py task:=task2 noise:=true`
started Nav2 successfully: controller, planner and navigator lifecycle states
were ACTIVE. However MPPI `planner_node` exited with code 1: normal ROS Python
could not import `torch`. `/planned_path_pruned` had zero publishers and
`/autonomy/ready` was false. The opponent-selector no-track warning was not the
startup cause. Earlier acceptance runs had used an explicitly exported temporary
Python dependency directory, which was missing from the interactive shell.

Copied the already available CPU environment using
`scripts/setup/install_task2_dependencies.py --from-directory /run/user/1000/njord-sim-deps`
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
python3 scripts/simulation/run_mission_sim.py --task task2 --noise --domain 126 \
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



### simulation_results/task2_persistent_torch_20261113/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 300, "domain": 126, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3789e309a76129bd4d0b62dc86da286b7e7151d1"}`

### topic_audit_20261113/task2_clearing/audit_repeat_port2028/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2028, "model": "kinematic", "timeout": 300, "domain": 141, "encounter": "port", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/clearing_only_head_on/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 300, "domain": 130, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/clearing_only_suite/task2-none-noise2026/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 300, "domain": 131, "encounter": "none", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/clearing_only_suite/task2-port-noise2026/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 300, "domain": 131, "encounter": "port", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/clearing_only_suite/task2-starboard-noise2026/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 300, "domain": 132, "encounter": "starboard", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/display_baseline_suite/task2-head_on-noise2026/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 300, "domain": 131, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/display_baseline_suite/task2-head_on-noise2027/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2027, "model": "kinematic", "timeout": 300, "domain": 132, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/display_baseline_suite/task2-head_on-noise2028/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2028, "model": "kinematic", "timeout": 300, "domain": 131, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/display_baseline_suite/task2-none-noise2026/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 300, "domain": 132, "encounter": "none", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/display_baseline_suite/task2-none-noise2027/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2027, "model": "kinematic", "timeout": 300, "domain": 131, "encounter": "none", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/display_baseline_suite/task2-none-noise2028/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2028, "model": "kinematic", "timeout": 300, "domain": 132, "encounter": "none", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/display_baseline_suite/task2-port-noise2026/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 300, "domain": 132, "encounter": "port", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/display_baseline_suite/task2-port-noise2027/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2027, "model": "kinematic", "timeout": 300, "domain": 131, "encounter": "port", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/display_baseline_suite/task2-port-noise2028/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2028, "model": "kinematic", "timeout": 300, "domain": 132, "encounter": "port", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/display_baseline_suite/task2-starboard-noise2026/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 300, "domain": 131, "encounter": "starboard", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/display_baseline_suite/task2-starboard-noise2027/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2027, "model": "kinematic", "timeout": 300, "domain": 132, "encounter": "starboard", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/display_baseline_suite/task2-starboard-noise2028/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2028, "model": "kinematic", "timeout": 300, "domain": 131, "encounter": "starboard", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/final_suite/task2-head_on-noise2026/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 300, "domain": 139, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/final_suite/task2-head_on-noise2027/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2027, "model": "kinematic", "timeout": 300, "domain": 140, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/final_suite/task2-head_on-noise2028/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2028, "model": "kinematic", "timeout": 300, "domain": 140, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/final_suite/task2-none-noise2026/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 300, "domain": 139, "encounter": "none", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/final_suite/task2-none-noise2027/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2027, "model": "kinematic", "timeout": 300, "domain": 140, "encounter": "none", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/final_suite/task2-none-noise2028/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2028, "model": "kinematic", "timeout": 300, "domain": 139, "encounter": "none", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/final_suite/task2-port-noise2026/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 300, "domain": 139, "encounter": "port", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/final_suite/task2-port-noise2027/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2027, "model": "kinematic", "timeout": 300, "domain": 140, "encounter": "port", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/final_suite/task2-port-noise2028/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2028, "model": "kinematic", "timeout": 300, "domain": 139, "encounter": "port", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/final_suite/task2-starboard-noise2026/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 300, "domain": 140, "encounter": "starboard", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/final_suite/task2-starboard-noise2027/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2027, "model": "kinematic", "timeout": 300, "domain": 139, "encounter": "starboard", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/final_suite/task2-starboard-noise2028/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2028, "model": "kinematic", "timeout": 300, "domain": 140, "encounter": "starboard", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/straight_only_diagnostic_repeat/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2027, "model": "kinematic", "timeout": 300, "domain": 134, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/straight_only_suite_interrupted/README.md


Stopped after head-on seed 2027 collided again. Head-on 2028 also completed below the required hull gap (0.446 m versus 0.5 m), without contact. Completed and interrupted evaluations are retained with their original reason fields; interrupted/pending cases are not passes. A same-side 7 m candidate was added before the next matrix.



### topic_audit_20261113/task2_clearing/straight_only_suite_interrupted/task2-head_on-noise2026/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 300, "domain": 135, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/straight_only_suite_interrupted/task2-head_on-noise2027/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2027, "model": "kinematic", "timeout": 300, "domain": 136, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/straight_only_suite_interrupted/task2-head_on-noise2028/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2028, "model": "kinematic", "timeout": 300, "domain": 135, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/straight_only_suite_interrupted/task2-port-noise2026/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 300, "domain": 136, "encounter": "port", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/wide_only_suite_interrupted/README.md


Wider 7 m candidates alone did not resolve head-on seed 2028: 0.007 m hull gap and no progress for 45 s. Stopped pending jobs; explicitly interrupted evaluations are not counted as navigation failures or passes. Next change removes the pre-gate excursion, adds straight gate approaches/departures, and preserves lateral continuity on replan.



### topic_audit_20261113/task2_clearing/wide_only_suite_interrupted/task2-head_on-noise2026/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 300, "domain": 137, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/wide_only_suite_interrupted/task2-head_on-noise2027/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2027, "model": "kinematic", "timeout": 300, "domain": 138, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/wide_only_suite_interrupted/task2-head_on-noise2028/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2028, "model": "kinematic", "timeout": 300, "domain": 138, "encounter": "head_on", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/wide_only_suite_interrupted/task2-port-noise2026/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2026, "model": "kinematic", "timeout": 300, "domain": 137, "encounter": "port", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`

### topic_audit_20261113/task2_clearing/wide_only_suite_interrupted/task2-port-noise2027/run.json

`{"task": "task2", "noise": true, "disturbance": false, "seed": 2027, "model": "kinematic", "timeout": 300, "domain": 138, "encounter": "port", "cancel_after": 0.0, "dropout_after": 0.0, "commit": "3a9b682a1f65b62e8fa10f8672631b1cebe00231"}`
