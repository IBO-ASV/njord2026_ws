#!/usr/bin/env bash
# Intra-process pipeline on-device bench.
# Launches robot/lidar.launch.py with lidar_source:=bag (in-container mcap
# replayer -> pcl_det + GLIM, all intra-process) and measures the update
# frequency of the key topics plus CPU/GPU headroom via tegrastats.
#
# Usage: run_bench.sh [WARMUP_S] [MEASURE_S]
set -u

WARMUP_S="${1:-30}"
MEASURE_S="${2:-60}"

WS=/home/ibo/njord2026_ws
OUT="$WS/scripts/intra_process_bench/out"
mkdir -p "$OUT"
LAUNCH_LOG="$OUT/launch.log"
TEGRA_LOG="$OUT/tegrastats.log"
SUMMARY="$OUT/summary.txt"
: > "$SUMMARY"

log() { echo "[bench] $*" | tee -a "$SUMMARY"; }

set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u

# Topics to measure: LiDAR/GLIM side (replayer, pcl_det, GLIM) + ZED2i side
# (SDK image/points, GPU-perception+LiDAR-fusion detections).
if [ -n "${BENCH_TOPICS:-}" ]; then
  read -ra TOPICS <<< "$BENCH_TOPICS"
else
  TOPICS=(/livox/lidar /livox/imu /odom /buoy_detections /buoy_detections_3d /zed2i/left/image_rect /zed2i/points)
fi

cleanup() {
  log "tearing down..."
  [ -n "${TEGRA_PID:-}" ] && kill "$TEGRA_PID" 2>/dev/null
  [ -n "${LAUNCH_PID:-}" ] && kill -INT "$LAUNCH_PID" 2>/dev/null
  sleep 3
  pkill -f "component_container_mt" 2>/dev/null
  # Launched as a plain Node (not a component), so the container pkill above
  # misses it and a stale one survives into the next sweep config.
  pkill -f "robot_state_publisher" 2>/dev/null
  pkill -f "bag_replayer" 2>/dev/null
  pkill -f "lidar.launch.py" 2>/dev/null
  pkill -f "combined_bench.launch.py" 2>/dev/null
  pkill -f "zed2i.launch.py" 2>/dev/null
  pkill -f "ground_video" 2>/dev/null
  sleep 1
}
trap cleanup EXIT

# X11 is available on this machine (:1, NVIDIA direct rendering), so the bench
# runs with GLIM's GUI viewers on by default -- that is the configuration the
# boat actually runs. Set BENCH_DISPLAY= (empty) to force a headless run.
export DISPLAY="${BENCH_DISPLAY-:1}"
[ -z "$DISPLAY" ] && unset DISPLAY
log "DISPLAY=${DISPLAY:-<headless>}"
COMBINED_LAUNCH="${BENCH_LAUNCH:-$WS/scripts/intra_process_bench/combined_bench.launch.py}"
# Extra `key:=value` launch args (space-separated) for config sweeps.
read -ra LAUNCH_ARGS <<< "${BENCH_LAUNCH_ARGS:-}"
log "launch: ros2 launch $COMBINED_LAUNCH ${LAUNCH_ARGS[*]}"
stdbuf -oL -eL ros2 launch "$COMBINED_LAUNCH" "${LAUNCH_ARGS[@]}" \
  > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

log "warmup ${WARMUP_S}s (GLIM/CUDA init)..."
sleep "$WARMUP_S"

if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
  log "ERROR: launch process died during warmup. Tail of launch log:"
  tail -n 40 "$LAUNCH_LOG" | tee -a "$SUMMARY"
  exit 1
fi

log "=== nodes ==="
ros2 node list 2>/dev/null | tee -a "$SUMMARY"
log "=== component list ==="
ros2 component list 2>/dev/null | tee -a "$SUMMARY"

# tegrastats over the measurement window.
log "starting tegrastats (${MEASURE_S}s)..."
( tegrastats --interval 1000 > "$TEGRA_LOG" 2>&1 ) &
TEGRA_PID=$!

# Measure all topics concurrently for one shared window.
log "measuring ${#TOPICS[@]} topics for ${MEASURE_S}s..."
declare -A HZFILE
HZ_PIDS=()
for t in "${TOPICS[@]}"; do
  f="$OUT/hz_$(echo "$t" | tr '/' '_').txt"
  HZFILE[$t]="$f"
  ( timeout "$MEASURE_S" ros2 topic hz "$t" > "$f" 2>&1 ) &
  HZ_PIDS+=($!)
done
# Wait ONLY on the bounded hz jobs (each self-terminates via `timeout`).
# tegrastats has no timeout, so never `wait` on it or the script hangs forever.
for p in "${HZ_PIDS[@]}"; do wait "$p" 2>/dev/null; done
kill "$TEGRA_PID" 2>/dev/null
wait "$TEGRA_PID" 2>/dev/null

log ""
log "================ FREQUENCY RESULTS ================"
for t in "${TOPICS[@]}"; do
  f="${HZFILE[$t]}"
  line=$(grep "average rate" "$f" | tail -n 1)
  det=$(grep -A1 "average rate" "$f" | grep -E "min:|max:|std dev" | tail -n 1)
  if [ -z "$line" ]; then
    log "$t : NO DATA (not publishing?)"
  else
    log "$t : $line | $det"
  fi
done

log ""
log "================ CPU / GPU (tegrastats) ================"
python3 - "$TEGRA_LOG" <<'PY' | tee -a "$SUMMARY"
import re, sys
lines = [l for l in open(sys.argv[1]) if 'RAM' in l]
if not lines:
    print("no tegrastats samples"); sys.exit()
cpu_busy=[]; gpu=[]; ram=[]
for l in lines:
    m=re.search(r'CPU \[([^\]]+)\]', l)
    if m:
        vals=[]
        for c in m.group(1).split(','):
            mm=re.match(r'(\d+)%', c.strip())
            if mm: vals.append(int(mm.group(1)))
        if vals: cpu_busy.append(sum(vals)/len(vals))
    g=re.search(r'GR3D_FREQ (\d+)%', l)
    if g: gpu.append(int(g.group(1)))
    r=re.search(r'RAM (\d+)/(\d+)MB', l)
    if r: ram.append((int(r.group(1)), int(r.group(2))))
def stat(x): return (sum(x)/len(x), max(x)) if x else (0,0)
ca,cp=stat(cpu_busy); ga,gp=stat(gpu)
print(f"samples: {len(lines)}")
print(f"CPU avg-busy(all 6 cores mean): mean={ca:.0f}%  peak={cp:.0f}%   (headroom mean={100-ca:.0f}%)")
print(f"GPU GR3D: mean={ga:.0f}%  peak={gp:.0f}%")
if ram: print(f"RAM: {ram[-1][0]}/{ram[-1][1]} MB (last), peak={max(r[0] for r in ram)} MB")
PY

log ""
log "bench done. logs: $LAUNCH_LOG  $TEGRA_LOG  $OUT/hz_*.txt"
