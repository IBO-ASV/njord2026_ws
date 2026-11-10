#!/usr/bin/env bash
# ZED-only heavy-load config sweep: find a config that holds ~15 Hz.
# Varies camera_resolution and elliptical-FOV size; points stride stays 1
# (heaviest, from zed_heavy_params.yaml). Each config runs the shared bench.
set -u

WS=/home/ibo/njord2026_ws
DIR="$WS/scripts/intra_process_bench"
OUT="$DIR/out"
RES="$OUT/sweep_results.txt"
mkdir -p "$OUT"
: > "$RES"

WARMUP="${1:-20}"
MEASURE="${2:-30}"

export BENCH_LAUNCH="$DIR/zed_heavy_bench.launch.py"
export BENCH_TOPICS="/zed2i/left/image_rect /zed2i/depth/image /zed2i/points /buoy_detections_3d"

# label | launch args
CONFIGS=(
  "HD1080_ellipse0.50|camera_resolution:=HD1080 framerate:=30 fov_ellipse_enable:=true fov_ellipse_a_ratio:=0.5 fov_ellipse_b_ratio:=0.5"
  "HD1080_ellipse0.35|camera_resolution:=HD1080 framerate:=30 fov_ellipse_enable:=true fov_ellipse_a_ratio:=0.35 fov_ellipse_b_ratio:=0.35"
  "HD720_ellipse0.50|camera_resolution:=HD720 framerate:=30 fov_ellipse_enable:=true fov_ellipse_a_ratio:=0.5 fov_ellipse_b_ratio:=0.5"
  "HD720_ellipse0.35|camera_resolution:=HD720 framerate:=30 fov_ellipse_enable:=true fov_ellipse_a_ratio:=0.35 fov_ellipse_b_ratio:=0.35"
)

for cfg in "${CONFIGS[@]}"; do
  label="${cfg%%|*}"
  args="${cfg#*|}"
  echo "======================================================================"
  echo "SWEEP CONFIG: $label   ($args)"
  export BENCH_LAUNCH_ARGS="$args"
  bash "$DIR/run_bench.sh" "$WARMUP" "$MEASURE"
  # snapshot this config's summary
  cp "$OUT/summary.txt" "$OUT/sweep_${label}.txt" 2>/dev/null
  {
    echo "### $label"
    grep -E "average rate|NO DATA|CPU %busy|CPU avg|GPU GR3D|RAM:" "$OUT/summary.txt" \
      | sed 's/\[bench\] //'
    echo ""
  } >> "$RES"
  # make sure nothing lingers between configs
  pkill -9 -x component_container_mt 2>/dev/null
  pkill -9 tegrastats 2>/dev/null
  sleep 3
done

echo "======================================================================"
echo "SWEEP DONE. Comparison written to $RES"
cat "$RES"
