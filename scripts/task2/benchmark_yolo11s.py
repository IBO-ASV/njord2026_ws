#!/usr/bin/env python3
"""Standalone YOLO11s inference benchmark for Task 2 (no ROS, headless).

Measures single-image inference latency for a .pt or .engine model:
N warmup runs followed by M timed runs, reporting mean / median / p95
latency and the resulting FPS. Runs on a synthetic image by default, or on
real images from a directory via --source.

Optionally samples `tegrastats` in a subprocess for the duration of the
timed runs (--tegrastats, Jetson only) and prints a GPU/CPU/RAM/temperature
summary afterwards.

Usage (inside the Jetson YOLO venv, see Docs/yolo11s_jetson_setup.md):

    python3 scripts/task2/benchmark_yolo11s.py \
        --model src/detection/yolo/config/yolo11s.engine \
        --imgsz 640 --warmup 10 --runs 100 --tegrastats

Never opens windows; safe to run over SSH.
"""

import argparse
import os
import re
import statistics
import subprocess
import sys
import time


IMAGE_EXTENSIONS = ('.jpg', '.jpeg', '.png', '.bmp')


def parse_args():
    parser = argparse.ArgumentParser(
        description='Standalone YOLO11s benchmark (.pt or .engine), no ROS.')
    parser.add_argument('--model', required=True,
                        help='Path to YOLO11s .pt weights or .engine file')
    parser.add_argument('--imgsz', type=int, default=640,
                        help='Inference image size (default: 640). Must match '
                             'the export imgsz for an engine.')
    parser.add_argument('--warmup', type=int, default=10,
                        help='Number of warmup inferences (default: 10)')
    parser.add_argument('--runs', type=int, default=100,
                        help='Number of timed inferences (default: 100)')
    parser.add_argument('--source', default=None,
                        help='Optional directory of images to benchmark on '
                             '(cycled through). Default: a synthetic random '
                             'image.')
    parser.add_argument('--device', default=None,
                        help='Inference device, e.g. "cuda:0" or "cpu". '
                             'Default: cuda:0 if available, else cpu. '
                             'Ignored for .engine.')
    parser.add_argument('--conf', type=float, default=0.25,
                        help='Confidence threshold (default: 0.25)')
    parser.add_argument('--half', action='store_true', default=False,
                        help='FP16 inference for the PyTorch backend on CUDA')
    parser.add_argument('--tegrastats', action='store_true', default=False,
                        help='Sample tegrastats during the timed runs '
                             '(Jetson only) and print a summary')
    return parser.parse_args()


def percentile(sorted_values, p):
    """Simple percentile with linear interpolation; input must be sorted."""
    if not sorted_values:
        return float('nan')
    k = (len(sorted_values) - 1) * (p / 100.0)
    lo = int(k)
    hi = min(lo + 1, len(sorted_values) - 1)
    frac = k - lo
    return sorted_values[lo] * (1.0 - frac) + sorted_values[hi] * frac


def load_images(source, imgsz):
    """Return a list of BGR numpy images to cycle through while benchmarking."""
    import numpy as np

    if source is None:
        # Synthetic image with a fixed seed so runs are comparable.
        rng = np.random.default_rng(0)
        img = rng.integers(0, 256, size=(imgsz, imgsz, 3), dtype=np.uint8)
        return [img]

    import cv2

    if not os.path.isdir(source):
        raise FileNotFoundError(f'--source directory not found: {source}')

    paths = sorted(
        os.path.join(source, f) for f in os.listdir(source)
        if f.lower().endswith(IMAGE_EXTENSIONS)
    )
    if not paths:
        raise FileNotFoundError(
            f'No images ({", ".join(IMAGE_EXTENSIONS)}) found in {source}')

    images = []
    for path in paths:
        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            print(f'WARNING: failed to read {path}, skipping', file=sys.stderr)
            continue
        images.append(img)
    if not images:
        raise RuntimeError(f'Could not read any image from {source}')
    return images


class TegrastatsSampler:
    """Runs `tegrastats` in a subprocess and summarizes its output lines."""

    def __init__(self, interval_ms=500):
        self.interval_ms = interval_ms
        self.proc = None
        self.lines = []

    def start(self):
        try:
            self.proc = subprocess.Popen(
                ['tegrastats', '--interval', str(self.interval_ms)],
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        except FileNotFoundError:
            print('WARNING: tegrastats not found (not a Jetson, or not in '
                  'PATH). Skipping tegrastats sampling.', file=sys.stderr)
            self.proc = None

    def stop(self):
        if self.proc is None:
            return
        self.proc.terminate()
        try:
            out, _ = self.proc.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            out, _ = self.proc.communicate()
        self.lines = [ln for ln in (out or '').splitlines() if ln.strip()]
        self.proc = None

    def summary(self):
        """Parse the sampled tegrastats lines into min/mean/max stats."""
        if not self.lines:
            return None

        gpu, cpu, ram_used, ram_total, temps = [], [], [], [], {}
        for line in self.lines:
            # GPU load, e.g. "GR3D_FREQ 45%"
            m = re.search(r'GR3D_FREQ (\d+)%', line)
            if m:
                gpu.append(int(m.group(1)))
            # Per-core CPU loads, e.g. "CPU [12%@1420,5%@1420,...]"
            m = re.search(r'CPU \[([^\]]+)\]', line)
            if m:
                loads = [int(x) for x in re.findall(r'(\d+)%@', m.group(1))]
                if loads:
                    cpu.append(sum(loads) / len(loads))
            # RAM, e.g. "RAM 3421/7620MB"
            m = re.search(r'RAM (\d+)/(\d+)MB', line)
            if m:
                ram_used.append(int(m.group(1)))
                ram_total.append(int(m.group(2)))
            # Temperatures, e.g. "cpu@45.5C gpu@44.2C tj@46C"
            for name, val in re.findall(r'([A-Za-z_]\w*)@([\d.]+)C', line):
                temps.setdefault(name, []).append(float(val))

        def stats(vals):
            if not vals:
                return None
            return {'min': min(vals), 'mean': sum(vals) / len(vals),
                    'max': max(vals)}

        return {
            'samples': len(self.lines),
            'gpu_pct': stats(gpu),
            'cpu_pct': stats(cpu),
            'ram_used_mb': stats(ram_used),
            'ram_total_mb': ram_total[0] if ram_total else None,
            'temps_c': {k: stats(v) for k, v in temps.items()},
        }


def print_tegrastats_summary(summary):
    if summary is None:
        print('tegrastats  : no samples collected')
        return
    print(f'tegrastats  : {summary["samples"]} samples')

    def fmt(name, s, unit):
        if s is None:
            return f'  {name}: n/a'
        return (f'  {name}: mean {s["mean"]:.1f}{unit} '
                f'(min {s["min"]:.1f}, max {s["max"]:.1f})')

    print(fmt('GPU load', summary['gpu_pct'], '%'))
    print(fmt('CPU load', summary['cpu_pct'], '%'))
    ram = summary['ram_used_mb']
    if ram is not None:
        total = summary['ram_total_mb']
        print(f'  RAM used: mean {ram["mean"]:.0f} MB '
              f'(min {ram["min"]:.0f}, max {ram["max"]:.0f}) / {total} MB')
    for name, s in sorted(summary['temps_c'].items()):
        print(fmt(f'temp {name}', s, 'C'))


def main():
    args = parse_args()

    if not os.path.isfile(args.model):
        print(f'ERROR: model not found: {args.model}. Weights/engines are '
              'not shipped in the repository; see '
              'Docs/yolo11s_jetson_setup.md.', file=sys.stderr)
        return 1
    if args.runs <= 0:
        print('ERROR: --runs must be > 0', file=sys.stderr)
        return 1

    try:
        from ultralytics import YOLO
    except Exception as exc:  # noqa: BLE001
        print(f'ERROR: failed to import ultralytics: {exc}. Activate the '
              'YOLO venv first (see Docs/yolo11s_jetson_setup.md).',
              file=sys.stderr)
        return 1

    try:
        import torch
        cuda_available = torch.cuda.is_available()
    except Exception:  # noqa: BLE001
        torch = None
        cuda_available = False

    is_engine = args.model.endswith('.engine')
    device = args.device
    if device is None:
        device = 'cuda:0' if cuda_available else 'cpu'

    if is_engine and not cuda_available:
        print('ERROR: a TensorRT .engine requires CUDA. Run this on the '
              'Jetson the engine was exported on.', file=sys.stderr)
        return 1

    print(f'Model  : {args.model} '
          f'({"TensorRT engine" if is_engine else "PyTorch weights"})')
    print(f'Device : {"(engine)" if is_engine else device}')
    print(f'imgsz  : {args.imgsz}, conf: {args.conf}, half: {args.half}')

    try:
        images = load_images(args.source, args.imgsz)
    except (FileNotFoundError, RuntimeError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 1
    if args.source:
        print(f'Source : {len(images)} images from {args.source}')
    else:
        print(f'Source : synthetic random image ({args.imgsz}x{args.imgsz})')

    model = YOLO(args.model)

    predict_kwargs = {
        'imgsz': args.imgsz,
        'conf': args.conf,
        'verbose': False,
    }
    if not is_engine:
        predict_kwargs['device'] = device
        predict_kwargs['half'] = args.half and 'cuda' in str(device)

    # Warmup: the first calls include model init / engine deserialization.
    print(f'Warmup : {args.warmup} runs ...')
    for i in range(args.warmup):
        model.predict(images[i % len(images)], **predict_kwargs)

    sampler = None
    if args.tegrastats:
        sampler = TegrastatsSampler()
        sampler.start()

    print(f'Timing : {args.runs} runs ...')
    latencies_ms = []
    for i in range(args.runs):
        img = images[i % len(images)]
        t0 = time.perf_counter()
        model.predict(img, **predict_kwargs)
        if torch is not None and cuda_available:
            # Make sure async GPU work is included in the measured latency.
            torch.cuda.synchronize()
        latencies_ms.append((time.perf_counter() - t0) * 1000.0)

    if sampler is not None:
        sampler.stop()

    latencies_ms.sort()
    mean_ms = statistics.fmean(latencies_ms)
    median_ms = percentile(latencies_ms, 50)
    p95_ms = percentile(latencies_ms, 95)

    print()
    print('=== YOLO11s benchmark result ===')
    print(f'model       : {args.model}')
    print(f'runs        : {args.runs} (warmup {args.warmup})')
    print(f'mean        : {mean_ms:.2f} ms')
    print(f'median      : {median_ms:.2f} ms')
    print(f'p95         : {p95_ms:.2f} ms')
    print(f'min / max   : {latencies_ms[0]:.2f} / {latencies_ms[-1]:.2f} ms')
    print(f'FPS (mean)  : {1000.0 / mean_ms:.1f}')

    if args.tegrastats and sampler is not None:
        print()
        print_tegrastats_summary(sampler.summary())

    print()
    print('NOTE: when reporting Jetson numbers, also record the power mode '
          '(`sudo nvpmodel -q`) and whether `jetson_clocks` was active.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
