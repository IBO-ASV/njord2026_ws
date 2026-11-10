#!/usr/bin/env python3
"""Validate a TensorRT engine's one-input/one-output deployment contract."""

import argparse
import hashlib
import json
import os
import platform
from pathlib import Path
import sys


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("engine", type=Path)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--onnx", type=Path)
    args = parser.parse_args()
    try:
        import tensorrt as trt
    except ImportError as error:
        raise SystemExit(f"TensorRT Python bindings unavailable: {error}")
    logger = trt.Logger(trt.Logger.ERROR)
    runtime = trt.Runtime(logger)
    engine = runtime.deserialize_cuda_engine(args.engine.read_bytes())
    if engine is None:
        raise SystemExit("TensorRT could not deserialize engine")
    tensors = [{"name": engine.get_tensor_name(i), "shape": list(engine.get_tensor_shape(engine.get_tensor_name(i))),
                "dtype": str(engine.get_tensor_dtype(engine.get_tensor_name(i))),
                "mode": str(engine.get_tensor_mode(engine.get_tensor_name(i)))} for i in range(engine.num_io_tensors)]
    inputs = [item for item in tensors if item["mode"].endswith("INPUT")]
    outputs = [item for item in tensors if item["mode"].endswith("OUTPUT")]
    if len(inputs) != 1 or len(outputs) != 1 or inputs[0]["shape"] != [1, 3, 640, 640]:
        raise SystemExit(f"Unexpected TensorRT contract: {tensors}")
    output_shape = outputs[0]["shape"]
    if len(output_shape) != 3 or 6 not in output_shape:
        raise SystemExit(f"Unexpected decoded output shape: {output_shape}")
    digest = hashlib.sha256(args.engine.read_bytes()).hexdigest()
    manifest = {
        "tensorrt_version": trt.__version__,
        "cuda_version": os.environ.get("CUDA_VERSION", "unknown"),
        "gpu": os.environ.get("GPU_NAME", platform.machine()),
        "engine_sha256": digest,
        "onnx_sha256": hashlib.sha256(args.onnx.read_bytes()).hexdigest() if args.onnx else None,
        "tensors": tensors,
    }
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
