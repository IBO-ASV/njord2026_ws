#!/usr/bin/env python3
"""Export YOLO11s weights (.pt) to a TensorRT engine (.engine) on Jetson.

This script must run ON THE JETSON, never on a development Mac/PC without
CUDA: a TensorRT engine is specific to the GPU and TensorRT version it was
built with, so it has to be exported on the exact device that will run it.
The script refuses to run when CUDA is unavailable.

Usage (with the Jetson YOLO venv activated, see Docs/yolo11s_jetson_setup.md):

    python3 scripts/yolo/export_yolo11s_tensorrt.py \
        --weights src/detection/yolo/config/yolo11s.pt \
        --imgsz 640 \
        --output src/detection/yolo/config/yolo11s.engine

FP16 is enabled by default; disable with --no-half.
INT8 requires --calib-data (a calibration dataset); it is refused otherwise
because an uncalibrated INT8 engine has unvalidated accuracy.

The environment (torch / ultralytics / tensorrt / CUDA / JetPack versions)
is recorded to a sidecar .json next to the engine for reproducibility.
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
from datetime import datetime, timezone


def read_jetpack_release():
    """Return JetPack / L4T info from /etc/nv_tegra_release if present."""
    path = '/etc/nv_tegra_release'
    if not os.path.exists(path):
        return None
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as f:
            return f.read().strip()
    except OSError:
        return None


def read_jetson_release_cmd():
    """Return `jetson_release` output if the tool is installed (jetson-stats)."""
    exe = shutil.which('jetson_release')
    if exe is None:
        return None
    try:
        out = subprocess.run(
            [exe, '-s'], capture_output=True, text=True, timeout=20)
        return (out.stdout or out.stderr).strip() or None
    except Exception:  # noqa: BLE001
        return None


def collect_environment():
    env = {
        'exported_at': datetime.now(timezone.utc).isoformat(),
        'python': sys.version,
        'platform': sys.platform,
        'nv_tegra_release': read_jetpack_release(),
        'jetson_release': read_jetson_release_cmd(),
    }

    try:
        import torch
        env['torch'] = torch.__version__
        env['cuda'] = torch.version.cuda
        env['cuda_available'] = torch.cuda.is_available()
        if torch.cuda.is_available():
            env['cuda_device_name'] = torch.cuda.get_device_name(0)
    except Exception as exc:  # noqa: BLE001
        env['torch'] = f'unavailable: {exc}'

    try:
        import ultralytics
        env['ultralytics'] = ultralytics.__version__
    except Exception as exc:  # noqa: BLE001
        env['ultralytics'] = f'unavailable: {exc}'

    try:
        import tensorrt
        env['tensorrt'] = tensorrt.__version__
    except Exception as exc:  # noqa: BLE001
        env['tensorrt'] = f'unavailable: {exc}'

    return env


def parse_args():
    parser = argparse.ArgumentParser(
        description='Export YOLO11s .pt weights to a TensorRT .engine '
                    '(Jetson only, requires CUDA).')
    parser.add_argument('--weights', required=True,
                        help='Path to trained YOLO11s .pt weights (required)')
    parser.add_argument('--imgsz', type=int, default=640,
                        help='Inference image size baked into the engine '
                             '(default: 640). Must match the imgsz used by '
                             'the node.')
    parser.add_argument('--half', dest='half', action='store_true', default=True,
                        help='Build an FP16 engine (default: enabled)')
    parser.add_argument('--no-half', dest='half', action='store_false',
                        help='Disable FP16 (build an FP32 engine)')
    parser.add_argument('--int8', action='store_true', default=False,
                        help='Build an INT8 engine. Requires --calib-data; '
                             'accuracy is UNVALIDATED without proper '
                             'calibration data.')
    parser.add_argument('--calib-data', default=None,
                        help='Calibration dataset (dataset YAML or image dir) '
                             'for INT8, required with --int8')
    parser.add_argument('--workspace', type=float, default=4.0,
                        help='TensorRT builder workspace size in GB '
                             '(default: 4.0)')
    parser.add_argument('--output', default=None,
                        help='Output path for the .engine file. Default: '
                             'next to the input weights '
                             '(<weights dir>/<weights name>.engine).')
    parser.add_argument('--device', default='0',
                        help='CUDA device index for the export (default: 0)')
    return parser.parse_args()


def main():
    args = parse_args()

    # --- Hard refusals ----------------------------------------------------
    if sys.platform == 'darwin':
        print('ERROR: this script must be run on the Jetson, not on macOS. '
              'TensorRT engines are device-specific and macOS has no CUDA.',
              file=sys.stderr)
        return 1

    try:
        import torch
    except Exception as exc:  # noqa: BLE001
        print(f'ERROR: failed to import torch: {exc}. Activate the Jetson '
              'YOLO venv first (see Docs/yolo11s_jetson_setup.md).',
              file=sys.stderr)
        return 1

    if not torch.cuda.is_available():
        print('ERROR: CUDA is not available on this machine. A TensorRT '
              'engine can only be built on the CUDA device that will run '
              'it (the Jetson). Refusing to export.', file=sys.stderr)
        return 1

    if args.int8 and not args.calib_data:
        print('ERROR: --int8 requires --calib-data (calibration dataset). '
              'Without calibration data the INT8 engine accuracy cannot be '
              'validated, so the export is refused.', file=sys.stderr)
        return 1

    if not os.path.isfile(args.weights):
        print(f'ERROR: weights not found: {args.weights}. Weights are not '
              'shipped in the repository; see Docs/yolo11s_jetson_setup.md.',
              file=sys.stderr)
        return 1

    if not args.weights.endswith('.pt'):
        print(f'ERROR: --weights must be a .pt file, got: {args.weights}',
              file=sys.stderr)
        return 1

    if args.calib_data and not os.path.exists(args.calib_data):
        print(f'ERROR: calib data not found: {args.calib_data}',
              file=sys.stderr)
        return 1

    # --- Resolve output ----------------------------------------------------
    if args.output:
        output_path = os.path.abspath(args.output)
        if not output_path.endswith('.engine'):
            print(f'ERROR: --output must end with .engine, got: {args.output}',
                  file=sys.stderr)
            return 1
        os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
    else:
        output_path = os.path.abspath(
            os.path.splitext(args.weights)[0] + '.engine')

    try:
        from ultralytics import YOLO
    except Exception as exc:  # noqa: BLE001
        print(f'ERROR: failed to import ultralytics: {exc}. Activate the '
              'Jetson YOLO venv first (see Docs/yolo11s_jetson_setup.md).',
              file=sys.stderr)
        return 1

    # --- Export -------------------------------------------------------------
    model = YOLO(args.weights)

    export_kwargs = {
        'format': 'engine',
        'imgsz': args.imgsz,
        'half': args.half and not args.int8,
        'int8': args.int8,
        'workspace': args.workspace,
        'device': args.device,
    }
    if args.int8:
        export_kwargs['data'] = args.calib_data
        print('WARNING: building an INT8 engine. Accuracy is UNVALIDATED '
              'until it has been checked against real course footage; keep '
              'the FP16 engine as fallback.')

    print(f'Exporting with: {export_kwargs}')
    engine_path = str(model.export(**export_kwargs))
    print(f'Export finished: {engine_path}')

    # Ultralytics writes the .engine next to the weights; move it to --output.
    if os.path.abspath(engine_path) != output_path:
        shutil.move(engine_path, output_path)
    engine_path = output_path

    # --- Sidecar environment record (next to the engine) --------------------
    env = collect_environment()
    env['args'] = {
        'weights': os.path.abspath(args.weights),
        'imgsz': args.imgsz,
        'half': args.half and not args.int8,
        'int8': args.int8,
        'calib_data': args.calib_data,
        'workspace_gb': args.workspace,
        'device': args.device,
    }
    env['engine'] = engine_path
    sidecar_path = engine_path + '.json'
    with open(sidecar_path, 'w', encoding='utf-8') as f:
        json.dump(env, f, indent=2, ensure_ascii=False)

    print(f'TensorRT engine : {engine_path}')
    print(f'Environment log : {sidecar_path}')
    print()
    print('Next steps:')
    print(f'  1. Sanity-check the engine: python3 scripts/task2/'
          f'benchmark_yolo11s.py --model {engine_path} --imgsz {args.imgsz}')
    print('  2. Place/keep the engine where the node expects it, e.g. '
          'src/detection/yolo/config/yolo11s.engine, then colcon build.')
    print('  3. Launch: ros2 launch yolo yolo11s.launch.py '
          f'yolo_model_path:={engine_path} backend:=tensorrt')
    print('  4. Re-export after any JetPack / TensorRT upgrade: engines do '
          'not survive version changes.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
