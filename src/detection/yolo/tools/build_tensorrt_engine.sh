#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "usage: $0 INPUT.onnx OUTPUT.engine" >&2
  exit 2
fi
onnx=$1
engine=$2
if [[ ! -r "$onnx" ]]; then
  echo "input ONNX does not exist or is not readable: $onnx" >&2
  exit 1
fi
command -v trtexec >/dev/null || { echo "trtexec was not found in PATH" >&2; exit 1; }
mkdir -p "$(dirname "$engine")"
log="${engine}.build.log"
manifest="${engine}.manifest.json"
shape_spec="$(python3 - "$onnx" <<'PY'
import sys
import onnx
model = onnx.load(sys.argv[1])
if len(model.graph.input) != 1:
    raise SystemExit("ONNX must have exactly one input")
input_tensor = model.graph.input[0]
dimensions = input_tensor.type.tensor_type.shape.dim
if any(not dim.HasField("dim_value") for dim in dimensions):
    shape = "x".join(
        str(dim.dim_value) if dim.HasField("dim_value") else "1"
        for dim in dimensions
    )
    print(f"{input_tensor.name}:{shape}")
PY
)"

trtexec_args=(--onnx="$onnx" --saveEngine="$engine" --fp16)
if [[ -n "$shape_spec" ]]; then
  trtexec_args+=(
    "--minShapes=${shape_spec}"
    "--optShapes=${shape_spec}"
    "--maxShapes=${shape_spec}"
  )
fi
trtexec "${trtexec_args[@]}" 2>&1 | tee "$log"
"$(dirname "$0")/inspect_tensorrt_engine.py" "$engine" --manifest "$manifest" --onnx "$onnx"
