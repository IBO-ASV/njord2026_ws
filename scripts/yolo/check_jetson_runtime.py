#!/usr/bin/env python3
"""Sensor-free Python/ROS/GPU verification, including actual YOLO inference."""

import argparse
import importlib.metadata
import json
import os
import platform
import sys
import time
from datetime import datetime, timezone
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", choices=["cpu", "cuda:0"], default="cuda:0")
    parser.add_argument(
        "--model",
        type=Path,
        default=Path(__file__).resolve().parents[2]
        / "src/robot/config/yolo_model/best.pt",
    )
    parser.add_argument(
        "--ros", action="store_true", help="also exercise cv_bridge and rclpy"
    )
    parser.add_argument(
        "--triton", action="store_true", help="also compile and run a small GPU kernel"
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = {
        "time": datetime.now(timezone.utc).isoformat(),
        "python": sys.executable,
        "platform": platform.machine(),
        "requested_device": args.device,
        "passed": False,
    }
    config = Path(os.environ.get("XDG_RUNTIME_DIR", "/tmp")) / "njord-yolo-check"
    config.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("YOLO_CONFIG_DIR", str(config))
    try:
        import cv2
        import matplotlib

        matplotlib.use("Agg")
        import numpy as np
        import polars as pl
        import torch
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure
        from scipy.linalg import solve
        from torchvision.ops import nms
        from ultralytics import YOLO

        names = [
            "numpy",
            "scipy",
            "opencv-python",
            "matplotlib",
            "polars",
            "torch",
            "torchvision",
            "ultralytics",
        ]
        report["versions"] = {name: importlib.metadata.version(name) for name in names}
        report["cuda_build"] = torch.version.cuda
        report["cuda_available"] = torch.cuda.is_available()
        if args.device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError(
                "CUDA is unavailable: use uv sync --extra jetson, not a PyPI/CPU torch build"
            )
        if args.device.startswith("cuda"):
            report["gpu"] = torch.cuda.get_device_name()
            report["compute_capability"] = torch.cuda.get_device_capability()
        assert np.allclose(solve([[2.0, 0.0], [0.0, 2.0]], [2.0, 4.0]), [1.0, 2.0])
        image = np.zeros((320, 320, 3), dtype=np.uint8)
        assert cv2.resize(image, (160, 160)).shape == (160, 160, 3)
        assert pl.DataFrame({"a": [1, 2]}).select(pl.col("a").sum()).item() == 3
        figure = Figure()
        figure.add_subplot().plot([0, 1], [0, 1])
        FigureCanvasAgg(figure).draw()
        report["numeric_image_plot_checks"] = True
        matrix = torch.ones((128, 128), device=args.device)
        assert (matrix @ matrix)[0, 0].item() == 128
        boxes = torch.tensor(
            [[0, 0, 10, 10], [1, 1, 9, 9], [20, 20, 30, 30]],
            dtype=torch.float32,
            device=args.device,
        )
        scores = torch.tensor([0.9, 0.8, 0.7], device=args.device)
        assert nms(boxes, scores, 0.5).tolist() == [0, 2]
        report["tensor_and_nms_checks"] = True
        if args.triton:
            if args.device == "cpu":
                raise RuntimeError("--triton requires cuda:0")
            import triton
            import triton.language as tl

            @triton.jit
            def add_one(x, y, n: tl.constexpr, BLOCK: tl.constexpr):
                offsets = tl.program_id(0) * BLOCK + tl.arange(0, BLOCK)
                values = tl.load(x + offsets, offsets < n, other=0)
                tl.store(y + offsets, values + 1, offsets < n)

            x = torch.arange(257, device=args.device, dtype=torch.float32)
            y = torch.empty_like(x)
            add_one[(triton.cdiv(x.numel(), 128),)](x, y, x.numel(), 128)
            torch.testing.assert_close(y, x + 1)
            report["triton_kernel"] = True
        if args.ros:
            import rclpy
            from cv_bridge import CvBridge

            bridge = CvBridge()
            message = bridge.cv2_to_imgmsg(image, encoding="bgr8")
            np.testing.assert_array_equal(bridge.imgmsg_to_cv2(message, "bgr8"), image)
            report["ros_cv_bridge_roundtrip"] = True
            report["rclpy"] = rclpy.__file__
        if not args.model.is_file():
            raise FileNotFoundError(
                f"Local model is missing: {args.model}; no model is downloaded"
            )
        model = YOLO(str(args.model))
        timings = []
        for _ in range(3):
            start = time.perf_counter()
            results = model.predict(
                image, device=args.device, imgsz=320, verbose=False, save=False
            )
            if args.device.startswith("cuda"):
                torch.cuda.synchronize()
            timings.append((time.perf_counter() - start) * 1000)
        actual = str(next(model.model.parameters()).device)
        if actual != args.device:
            raise RuntimeError(f"YOLO executed on {actual}, expected {args.device}")
        report["yolo"] = {
            "model": str(args.model.resolve()),
            "device": actual,
            "classes": model.names,
            "detections": len(results[0].boxes),
            "timing_ms": timings,
            "input": "synthetic black 320x320 BGR; not an accuracy test",
        }
        report["passed"] = True
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
