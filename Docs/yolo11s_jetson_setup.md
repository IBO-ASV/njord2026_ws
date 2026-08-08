# YOLO11s on Jetson Orin Nano — environment setup and operation

How to run the new `yolo11_node` (standard Ultralytics YOLO11s detector for
Njord Task 2 buoy detection) on the Jetson Orin Nano Super. The node is a
parallel implementation next to the existing `yolo_node` (YOLOv10-fork based)
and does not change that node or its environment.

## Environment strategy

| Component | Where it lives | Why |
|---|---|---|
| ROS 2 / cv_bridge / ZED SDK | System Python | Must stay ABI-compatible with the apt-managed ROS install; never goes into a venv |
| ultralytics (YOLO) | Dedicated venv created with `--system-site-packages` | Keeps the system Python clean while still seeing cv_bridge etc. |
| torch / torchvision | NVIDIA-provided JetPack wheels, installed manually | **NEVER overwrite with PyPI torch via pip** — the PyPI build is CPU-only on Jetson and silently kills CUDA. They are deliberately absent from the requirements file |
| numpy | Pinned `>=1.26,<2.0` | numpy 2.x breaks cv_bridge and the system ROS Python stack |
| ultralytics version | Pinned `8.3.237` | Matches the version validated in the repo's main `requirements.txt` |

The minimal venv package list is `src/detection/yolo/requirements_jetson_yolo11.txt`.

## Creating the venv (on the Jetson)

```bash
# 1. Source the system ROS 2 first
source /opt/ros/humble/setup.bash

# 2. Create the dedicated venv with --system-site-packages
#    (makes system packages like cv_bridge visible inside the venv)
python3 -m venv ~/venvs/yolo11 --system-site-packages
source ~/venvs/yolo11/bin/activate

# 3. Install the Jetson torch (NVIDIA wheel) FIRST.
#    Pick the wheel matching the installed JetPack version from:
#    https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/
#    e.g.: pip install --no-cache <nvidia-torch-wheel-url>
#    WARNING: from this point on, never run a plain `pip install torch`.

# 4. Install the YOLO packages (torch/torchvision intentionally excluded)
pip install -r src/detection/yolo/requirements_jetson_yolo11.txt

# 5. Verify nothing got clobbered
python3 -c "import numpy; print(numpy.__version__)"                     # must be 1.26.x
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"  # must be True
```

If a later `pip install` tries to upgrade numpy or replace torch, abort it and
restore with `pip install "numpy>=1.26,<2"` / reinstall the NVIDIA torch wheel.

## Obtaining and placing the YOLO11s weights

The weights are **NOT in the repository** and are never downloaded
automatically by the node. You must train (or be given) a YOLO11s model for
the Njord buoy classes and place it yourself. The nominal path is:

```
src/detection/yolo/config/yolo11s.pt
```

```bash
cp /path/to/your/trained/yolo11s.pt src/detection/yolo/config/yolo11s.pt
colcon build --packages-select yolo    # config/ is installed into the share dir
```

Alternatively keep the file anywhere and pass its absolute path via the
`model_path` parameter or the `yolo_model_path` launch argument. If the file
does not exist, the node logs a fatal error naming the expected path and
shuts down cleanly.

## Exporting a TensorRT engine (optional, faster)

Run **on the Jetson only** — engines are specific to the GPU and TensorRT
version they were built with; the export script refuses to run without CUDA.

```bash
source ~/venvs/yolo11/bin/activate
python3 scripts/yolo/export_yolo11s_tensorrt.py \
    --weights src/detection/yolo/config/yolo11s.pt \
    --imgsz 640 \
    --output src/detection/yolo/config/yolo11s.engine
```

- FP16 is the default (`--no-half` for FP32).
- INT8 (`--int8`) additionally requires `--calib-data`; without calibration
  data the export is refused because INT8 accuracy would be unvalidated.
  Treat any INT8 engine as unvalidated until checked against real footage.
- A sidecar `<engine>.json` records torch/ultralytics/tensorrt/CUDA/JetPack
  versions for reproducibility.
- Re-export after every JetPack / TensorRT upgrade.

## Launching the node

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source ~/venvs/yolo11/bin/activate    # required: the launch file has a venv guard

# PyTorch backend (.pt)
ros2 launch yolo yolo11s.launch.py \
    yolo_model_path:=/path/to/yolo11s.pt device:=cuda:0

# TensorRT backend (.engine)
ros2 launch yolo yolo11s.launch.py \
    yolo_model_path:=/path/to/yolo11s.engine backend:=tensorrt
```

Outputs: `/yolo/detections` (vision_msgs/Detection2DArray), `/buoy_roi`
(njord_interfaces/BuoyRoi), `yolo/debug_image` (rate-limited). All parameters
(thresholds, rate limits, ROI crop, class-name/color mapping) are documented
in `src/detection/yolo/config/yolo11s_params.yaml`; the values marked [TUNE]
there need tuning against real course footage.

## Benchmarking

```bash
source ~/venvs/yolo11/bin/activate
python3 scripts/task2/benchmark_yolo11s.py \
    --model src/detection/yolo/config/yolo11s.engine \
    --imgsz 640 --warmup 10 --runs 100 --tegrastats
```

`--tegrastats` samples GPU/CPU/RAM/temperature during the timed runs. When
reporting numbers, also record the power mode (`sudo nvpmodel -q`) and
whether `jetson_clocks` was active.
