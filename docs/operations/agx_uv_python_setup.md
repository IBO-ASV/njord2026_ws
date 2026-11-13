# AGX Orin: uv CPU / Jetson GPU environments

Validated 2026-11-13 on this AGX Orin (SM 8.7), L4T R39.2.1 / JetPack 7.2.1,
Ubuntu 24.04, Python 3.12.3, host CUDA 13.2.2 and driver 595.78.
The project `.venv` is isolated from the system/user Python used by ROS and Task2.

## Setup and operation

```bash
cd ~/njord2026_ws
# GPU profile, including numeric/image/plot, CUDA/NMS/Triton and local YOLO tests:
bash scripts/setup/setup_python_uv.sh jetson

# Make ROS modules visible and put the selected .venv ahead of user packages:
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source scripts/setup/export_python_path.sh
python3 scripts/yolo/check_jetson_runtime.py --ros --triton

# Run the existing standard Ultralytics ROS detector with the existing local model:
ros2 launch yolo yolo11s.launch.py \
  yolo_model_path:=$PWD/src/robot/config/yolo_model/best.pt \
  device:=cuda:0 backend:=pytorch
```

`best.pt` is the existing six-class buoy model, not an invented/downloaded
YOLO11s checkpoint. The generic local `yolo11n.pt` model was also tested on GPU.
The launch's default `yolo11s.pt` is not present; pass an existing path as above.
No camera or other sensor is required by the runtime checker: it uses a black
320x320 BGR image. Actual camera-topic inference needs a real or simulated image
publisher. The test verifies execution and interfaces, not detection accuracy.
Existing TensorRT engines were not regenerated or qualified by this change.

To switch the project environment to CPU:

```bash
bash scripts/setup/setup_python_uv.sh cpu
# Restore GPU when needed:
bash scripts/setup/setup_python_uv.sh jetson
```

The two extras are mutually exclusive. Their equivalent raw sync commands are
`uv sync --frozen --extra cpu` and `uv sync --frozen --extra jetson`.
Use the setup script when switching: ARM CPU and CUDA torchvision wheels share
version 0.24.1, so it explicitly reinstalls torchvision to select the right binary
and corrects the vendor metadata issue described below. `uv run` users must also
specify the selected extra, e.g. `uv run --frozen --extra jetson ...`; a plain
`uv sync`/`uv run` does not select the GPU profile. Direct `.venv/bin/python` and
the sourced ROS launch do not implicitly resync packages.

No global shell startup files, GPU drivers, system CUDA, ROS packages or the
persistent user-site CPU torch were replaced. Use another terminal with the
normal ROS setup for Task2 sim; there is no need to activate the YOLO `.venv`
for that simulator. Its existing CPU PyTorch remains 2.15.0+cpu.

## Locked versions and dependency changes

| Component | CPU profile | Jetson GPU profile |
|---|---|---|
| torch | 2.9.1+cpu | 2.9.1+cu130 |
| torchvision | 0.24.1 (CPU wheel) | 0.24.1 (CUDA 13 wheel) |
| ultralytics | 8.3.237 | 8.3.237 |
| numpy | 1.26.4 | 1.26.4 |
| opencv-python | 4.10.0.84 | 4.10.0.84 |
| scipy | 1.15.3 | 1.15.3 |

`pyproject.toml` uses explicit official CPU/CUDA 13 indexes for torch/torchvision
and conflicting `cpu` / `jetson` extras. Redundant, unconditional desktop CUDA 12
pins and a standalone Triton pin were removed; the selected PyTorch build now
supplies the exact matching CUDA/Triton transitive requirements. The regenerated
`uv.lock` includes all platform/extra alternatives, not a demand to install every
entry on this board. No arbitrary minimum-version upgrade of the common Python
packages was made. The selected GPU environment contains 52 distributions and
`uv pip check --python .venv/bin/python` passes.

The initial legacy-lock installation was completed first and confirmed its ARM
PyTorch is CPU-only. Its unnecessary CUDA 12 installations and their downloaded
package caches were then removed while selecting the GPU profile. The final
`.venv` is the Jetson profile. System Python was not synchronized by uv.

## cuSPARSELt vendor metadata correction

The PyPI `nvidia_cusparselt_cu13-0.8.0-...-manylinux2014_aarch64.whl` has an
AArch64 ELF shared library but its internal WHEEL declares
`Tag: py3-none-manylinux2014_sbsa`. That is not a standard wheel platform tag:
uv reports incompatibility and repeatedly reinstalls it. Its library loads
successfully on this board.

`scripts/yolo/normalize_cusparselt_metadata.py` corrects only this exact installed
0.8.0 tag after verifying the library's ELF64/AArch64 header. It updates the
WHEEL entry's hash/size in RECORD and atomically replaces both metadata files,
breaking uv-cache hardlinks first. The cached vendor artifact and every shared
library remain unchanged. Other versions and already corrected metadata are
untouched. A test proves that an x86 library cannot be relabelled and that cached
hardlinked metadata remains intact. After correction, both `uv pip check` and
`uv sync --frozen --extra jetson --dry-run` pass without planned changes.

## Verification and retained evidence

- Actual GPU matrix multiplication and torchvision NMS return expected values.
- A Triton kernel compiles and executes correctly on Orin.
- NumPy/SciPy calculations, OpenCV operations, Matplotlib Agg rendering and
  Polars calculations pass.
- `cv_bridge` BGR image -> ROS Image -> BGR conversion is byte-for-byte identical.
- The local six-class model and YOLO11n both execute on `cuda:0`.
- On isolated ROS domain 127, a synthetic Image reaches `Yolo11DetectorNode`;
  it emits Detection2DArray while the model parameters are on `cuda:0`. No
  camera, LiDAR, GNSS driver or real actuator was started. A second synthetic
  noise/circle test lowers confidence only in the isolated test process to
  exercise nonempty detections: five boxes are serialized and received. This
  does not alter runtime defaults and is not an accuracy result.
- CPU profile switching is verified, followed by restoring and verifying GPU.

JSON evidence is under [jetson runtime results](../validation/recorded_results.md).
Timings there are synthetic-input smoke checks, not real-scene throughput tests.

Sources used to select the current platform/build combination:

- [NVIDIA JetPack downloads/release versions](https://developer.nvidia.com/embedded/jetpack/downloads)
- [Ultralytics native JetPack 7.2 installation](https://docs.ultralytics.com/guides/nvidia-jetson/#run-on-jetpack-72)
- [Astral uv PyTorch optional CPU/CUDA sources](https://docs.astral.sh/uv/guides/integration/pytorch/#configuring-accelerators-with-optional-dependencies)

Regression validation: 78 control/mission/sim/metadata tests passed; CPU/GPU
extras are rejected when selected together. Final GPU resync dry-run plans no
changes, and the system-side CPU torch remains 2.15.0+cpu.
