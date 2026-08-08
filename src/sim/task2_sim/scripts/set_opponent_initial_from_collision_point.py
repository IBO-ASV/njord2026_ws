#!/usr/bin/env python3
from pathlib import Path
import math
import re
import sys
import shutil
import time

OWN_SPEED_MPS = 2.0 * 0.514444
OPP_SPEED_MPS = 2.5 * 0.514444

WS = Path.home() / "0_Njord/njord2026_ws"
CONFIG_DIR = WS / "src/sim/task2_sim/config"
TASK_CONFIG_DEFAULT = CONFIG_DIR / "task2_params.yaml"
OPPONENT_YAML_DEFAULT = CONFIG_DIR / "task2_opponent_sim.yaml"

POINT_KEYWORDS = [
    "buoy", "buoys", "red", "green",
    "gps", "point", "points",
    "gate", "waypoint", "waypoints",
    "start", "goal",
    "position", "pose",
]

def parse_number_list(s):
    nums = re.findall(r"[-+]?\d+(?:\.\d+)?", s)
    if len(nums) >= 2:
        return float(nums[0]), float(nums[1])
    return None

def parse_points_from_text(text):
    """
    以下を拾う:
      x: 1.0 / y: 2.0 ブロック
      position: [1.0, 2.0, 0.0]
      - [1.0, 2.0]
      red_buoys: [[...], [...]]
    """
    points = []

    # ------------------------------------------------------------
    # 1. x: / y: 形式
    # ------------------------------------------------------------
    blocks = re.split(r"\n(?=\s*-\s+|\s*[A-Za-z0-9_]+\s*:)", text)
    for block in blocks:
        mx = re.search(r"^\s*x\s*:\s*([-+]?\d+(?:\.\d+)?)\s*$", block, re.M)
        my = re.search(r"^\s*y\s*:\s*([-+]?\d+(?:\.\d+)?)\s*$", block, re.M)
        if mx and my:
            x = float(mx.group(1))
            y = float(my.group(1))
            meta = block.lower()
            points.append((x, y, meta, "xy-block"))

    # ------------------------------------------------------------
    # 2. 配列形式
    # ------------------------------------------------------------
    lines = text.splitlines()
    context = []

    for i, line in enumerate(lines):
        low = line.lower()
        stripped = line.strip()

        # 直近のkey行をcontextとして保持
        if ":" in stripped and not stripped.startswith("#"):
            key_part = stripped.split(":", 1)[0].lower()
            if any(k in key_part for k in POINT_KEYWORDS):
                context.append(key_part)
                context = context[-6:]

        recent_context = " ".join(context + [low])

        # 関係なさそうな配列は除外
        if "[" not in line or "]" not in line:
            continue
        if not any(k in recent_context for k in POINT_KEYWORDS):
            continue

        # 1行内に複数配列がある場合も拾う
        arrays = re.findall(r"\[([^\[\]]+)\]", line)
        for arr in arrays:
            parsed = parse_number_list(arr)
            if parsed is None:
                continue
            x, y = parsed
            points.append((x, y, recent_context, "array"))

    # ------------------------------------------------------------
    # 3. 重複除去
    # ------------------------------------------------------------
    unique = []
    seen = set()
    for x, y, meta, mode in points:
        key = (round(x, 6), round(y, 6), mode)
        if key in seen:
            continue
        seen.add(key)
        unique.append((x, y, meta, mode))

    return unique

def find_yaml_value(text, keys):
    for key in keys:
        m = re.search(rf"^\s*{re.escape(key)}\s*:\s*([-+]?\d+(?:\.\d+)?)\s*$", text, re.M)
        if m:
            return float(m.group(1)), key
    return None, None

def set_yaml_value(text, key, value):
    pat = rf"^(\s*{re.escape(key)}\s*:\s*)([-+]?\d+(?:\.\d+)?)\s*$"
    text2, n = re.subn(pat, rf"\g<1>{value:.6f}", text, flags=re.M)
    if n == 0:
        text2 = text.rstrip() + f"\n{key}: {value:.6f}\n"
    return text2

def pick_own_start_and_goal(points):
    start_candidates = []
    goal_candidates = []

    for x, y, meta, mode in points:
        if "start" in meta or re.search(r"id\s*:\s*[\"']?5[\"']?", meta):
            start_candidates.append((x, y, meta, mode))
        if "goal" in meta or re.search(r"id\s*:\s*[\"']?6[\"']?", meta):
            goal_candidates.append((x, y, meta, mode))

    if start_candidates and goal_candidates:
        s = start_candidates[0]
        g = goal_candidates[0]
        return (s[0], s[1]), (g[0], g[1]), "start/goal or id 5/6"

    # 既知のTask2 GPS: 以前確認済みの 5=(0,0), 6=(60,0) を優先
    has_00 = any(abs(x) < 1e-6 and abs(y) < 1e-6 for x, y, _, _ in points)
    has_60 = any(abs(x - 60.0) < 1e-6 and abs(y) < 1e-6 for x, y, _, _ in points)

    if has_00 and has_60:
        return (0.0, 0.0), (60.0, 0.0), "known GPS 5/6 fallback"

    # 最後のフォールバック: x最小からx最大
    pts = sorted(points, key=lambda p: p[0])
    return (pts[0][0], pts[0][1]), (pts[-1][0], pts[-1][1]), "min-x to max-x fallback"

def pick_collision_point(points, own_start, own_goal):
    buoy_points = []

    for x, y, meta, mode in points:
        if ("buoy" in meta) or ("red" in meta) or ("green" in meta):
            if "start" in meta or "goal" in meta:
                continue
            buoy_points.append((x, y, meta, mode))

    if len(buoy_points) >= 2:
        xs = [p[0] for p in buoy_points]
        ys = [p[1] for p in buoy_points]
        return sum(xs) / len(xs), sum(ys) / len(ys), len(buoy_points), "buoy/red/green centroid"

    # ブイ名が拾えない場合は、自船start/goal以外の点の重心
    sx, sy = own_start
    gx, gy = own_goal
    rest = []

    for x, y, meta, mode in points:
        if math.hypot(x - sx, y - sy) < 1e-6:
            continue
        if math.hypot(x - gx, y - gy) < 1e-6:
            continue
        rest.append((x, y, meta, mode))

    if rest:
        xs = [p[0] for p in rest]
        ys = [p[1] for p in rest]
        return sum(xs) / len(xs), sum(ys) / len(ys), len(rest), "non-start/goal centroid"

    return (sx + gx) / 2.0, (sy + gy) / 2.0, 2, "start-goal midpoint fallback"

def main():
    task_config = Path(sys.argv[1]) if len(sys.argv) >= 2 else TASK_CONFIG_DEFAULT
    opponent_yaml = Path(sys.argv[2]) if len(sys.argv) >= 3 else OPPONENT_YAML_DEFAULT

    if not task_config.exists():
        print(f"[error] task config not found: {task_config}")
        sys.exit(1)

    if not opponent_yaml.exists():
        print(f"[error] opponent yaml not found: {opponent_yaml}")
        sys.exit(1)

    cfg_text = task_config.read_text()
    opp_text = opponent_yaml.read_text()

    points = parse_points_from_text(cfg_text)

    print(f"[use] task config   : {task_config}")
    print(f"[use] opponent yaml : {opponent_yaml}")
    print(f"[info] detected points: {len(points)}")

    for i, (x, y, meta, mode) in enumerate(points[:30]):
        meta_short = meta.replace("\t", " ").replace("  ", " ")[:80]
        print(f"  [{i:02d}] ({x:.3f}, {y:.3f}) mode={mode} meta={meta_short}")

    if len(points) < 2:
        print("[error] not enough points were detected from task2_params.yaml")
        print("[hint] show file with:")
        print(f"  nl -ba {task_config} | sed -n '1,220p'")
        sys.exit(1)

    own_start, own_goal, route_mode = pick_own_start_and_goal(points)
    cx, cy, n_used, col_mode = pick_collision_point(points, own_start, own_goal)

    heading_deg, heading_key = find_yaml_value(
        opp_text,
        ["heading_deg", "initial_heading_deg", "yaw_deg", "initial_yaw_deg", "heading"]
    )

    if heading_deg is None:
        own_yaw = math.degrees(math.atan2(own_goal[1] - own_start[1], own_goal[0] - own_start[0]))
        heading_deg = (own_yaw + 180.0) % 360.0
        heading_key = "heading_deg"
        print(f"[warn] heading key not found. use opposite own route: {heading_deg:.3f} deg")

    heading_rad = math.radians(heading_deg)
    hx = math.cos(heading_rad)
    hy = math.sin(heading_rad)

    dist_own = math.hypot(cx - own_start[0], cy - own_start[1])
    t_collision = dist_own / OWN_SPEED_MPS

    opp_x0 = cx - OPP_SPEED_MPS * t_collision * hx
    opp_y0 = cy - OPP_SPEED_MPS * t_collision * hy

    print("========== collision initial calculation ==========")
    print(f"own_start        = ({own_start[0]:.3f}, {own_start[1]:.3f})  mode={route_mode}")
    print(f"own_goal         = ({own_goal[0]:.3f}, {own_goal[1]:.3f})")
    print(f"collision point  = ({cx:.3f}, {cy:.3f})  mode={col_mode}, n={n_used}")
    print(f"own speed        = {OWN_SPEED_MPS:.6f} m/s")
    print(f"opp speed        = {OPP_SPEED_MPS:.6f} m/s")
    print(f"t_collision      = {t_collision:.3f} s")
    print(f"opp heading      = {heading_deg:.3f} deg  key={heading_key}")
    print(f"new opp initial  = ({opp_x0:.3f}, {opp_y0:.3f})")
    print("===================================================")

    backup = opponent_yaml.with_suffix(opponent_yaml.suffix + f".bak_collision_init_{time.strftime('%Y%m%d_%H%M%S')}")
    shutil.copy2(opponent_yaml, backup)
    print(f"[backup] {opponent_yaml} -> {backup}")

    x_val, x_key = find_yaml_value(opp_text, ["initial_x", "x0", "start_x", "x", "initial_pose_x"])
    y_val, y_key = find_yaml_value(opp_text, ["initial_y", "y0", "start_y", "y", "initial_pose_y"])

    if x_key is None:
        x_key = "initial_x"
        print("[warn] x key not found. append initial_x")
    if y_key is None:
        y_key = "initial_y"
        print("[warn] y key not found. append initial_y")

    opp_text = set_yaml_value(opp_text, x_key, opp_x0)
    opp_text = set_yaml_value(opp_text, y_key, opp_y0)

    speed_val, speed_key = find_yaml_value(opp_text, ["speed_mps", "speed", "velocity_mps"])
    if speed_key is not None:
        opp_text = set_yaml_value(opp_text, speed_key, OPP_SPEED_MPS)
        print(f"[set] {speed_key} = {OPP_SPEED_MPS:.6f}")

    opponent_yaml.write_text(opp_text)
    print(f"[write] {opponent_yaml}")
    print("[done] opponent initial position updated")

if __name__ == "__main__":
    main()
