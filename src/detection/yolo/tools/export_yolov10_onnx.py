#!/usr/bin/env python3
"""Export the fixed YOLOv10s TensorRT input contract and validate its ONNX graph."""

import argparse
import hashlib
import importlib.metadata
from pathlib import Path
import sys

CLASS_NAMES = ["green", "red", "north", "east", "south", "west"]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def shape(value_info):
    return [dim.dim_value for dim in value_info.type.tensor_type.shape.dim]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("model", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        from ultralytics import YOLO
        import onnx
    except ImportError as error:
        raise SystemExit(f"Missing export dependency: {error}")

    model = YOLO(str(args.model))
    exported = Path(model.export(format="onnx", imgsz=640, batch=1, dynamic=False, opset=17, simplify=True, half=False))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    exported.replace(args.output)
    graph = onnx.load(str(args.output)).graph
    if len(graph.input) != 1 or shape(graph.input[0]) != [1, 3, 640, 640]:
        raise SystemExit(f"Unexpected ONNX input contract: {[shape(item) for item in graph.input]}")
    if len(graph.output) != 1:
        raise SystemExit(f"Expected one ONNX output, found {len(graph.output)}")
    output_shape = shape(graph.output[0])
    if len(output_shape) != 3 or output_shape[0] != 1 or (output_shape[1] != 6 and output_shape[2] != 6):
        raise SystemExit(f"Expected decoded [1,N,6] or [1,6,N] output, got {output_shape}")
    names = [model.names[index] for index in range(len(model.names))]
    if names != CLASS_NAMES:
        raise SystemExit(f"Class order mismatch: expected {CLASS_NAMES}, got {names}")
    print(f"ultralytics={importlib.metadata.version('ultralytics')}")
    print(f"model_sha256={sha256(args.model)}")
    print(f"onnx_sha256={sha256(args.output)}")
    print(f"output_shape={output_shape}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
