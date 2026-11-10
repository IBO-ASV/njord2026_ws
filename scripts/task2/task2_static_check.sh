#!/usr/bin/env bash
# Task 2 static checks (no ROS 2 required).
#
# Run from anywhere; operates on the repository containing this script.
#   - py_compile of all Task2-related Python files
#   - YAML / XML parse of Task2 configs
#   - pytest of task2_perception and the selected mission/robot checks
#
# Each probe is best-effort (|| true) so the full report always prints;
# the exit code is non-zero if anything failed.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
cd "${REPO_ROOT}"

PASS_COUNT=0
FAIL_COUNT=0
RESULTS=()

report() {
    local status="$1"
    local label="$2"
    local detail="${3:-}"

    if [ "${status}" = "PASS" ]; then
        PASS_COUNT=$((PASS_COUNT + 1))
    else
        FAIL_COUNT=$((FAIL_COUNT + 1))
    fi

    RESULTS+=("[${status}] ${label}${detail:+: ${detail}}")
    printf '[%s] %s%s\n' "${status}" "${label}" "${detail:+: ${detail}}"
}

echo "=== Task 2 static checks (repo: ${REPO_ROOT}) ==="
echo

# ------------------------------------------------------------------
# 1. py_compile all Task2-related Python files
# ------------------------------------------------------------------
PY_FILES=()
while IFS= read -r f; do
    PY_FILES+=("$f")
done < <(
    {
        find src/navigation/path_generator/mppi -name '*.py' 2>/dev/null
        find src/navigation/path_generator/waypoint_publisher -name '*.py' 2>/dev/null
        find src/detection/task2_perception -name '*.py' 2>/dev/null
        find src/sim/task2_sim -name '*.py' 2>/dev/null
        find src -maxdepth 4 -type d -name 'task2_perception' 2>/dev/null \
            | while IFS= read -r d; do find "$d" -name '*.py' 2>/dev/null; done
        ls src/navigation/path_generator/mppi/launch/planner_real.launch.py 2>/dev/null
        ls src/detection/task2_perception/launch/task2_perception.launch.py 2>/dev/null
        ls src/detection/yolo/launch/yolo11s.launch.py 2>/dev/null
        ls src/robot/launch/navigation_launch_task2.py 2>/dev/null
        ls src/robot/launch/task2_mission_adapter.launch.py 2>/dev/null
        ls src/sim/task2_sim/launch/task2_sim.launch.py 2>/dev/null
        ls src/sim/task2_sim/task2_sim/cmd_vel_kinematic_sim.py 2>/dev/null
        ls scripts/task2/benchmark_mppi.py 2>/dev/null
        ls scripts/task2/benchmark_yolo11s.py 2>/dev/null
        ls scripts/yolo/export_yolo11s_tensorrt.py 2>/dev/null
    } | sort -u
)

if [ "${#PY_FILES[@]}" -eq 0 ]; then
    report FAIL "py_compile" "no Task2 Python files found"
else
    for f in "${PY_FILES[@]}"; do
        ERR="$(python3 -m py_compile "$f" 2>&1)" && \
            report PASS "py_compile ${f}" || \
            report FAIL "py_compile ${f}" "${ERR}" || true
    done
fi

# ------------------------------------------------------------------
# 2. YAML / XML parse of Task2 configs
# ------------------------------------------------------------------
YAML_FILES=()
while IFS= read -r f; do
    YAML_FILES+=("$f")
done < <(
    {
        ls src/robot/config/nav2_params_task2_*.yaml 2>/dev/null
        ls src/navigation/path_generator/mppi/config/mppi_params.yaml 2>/dev/null
        ls src/navigation/path_generator/waypoint_publisher/config/task2_waypoints.yaml 2>/dev/null
        find src -maxdepth 5 -path '*task2_perception*' -name '*.yaml' 2>/dev/null
    } | sort -u
)

for f in "${YAML_FILES[@]}"; do
    ERR="$(python3 -c "import yaml,sys; yaml.safe_load(open(sys.argv[1]))" "$f" 2>&1)" && \
        report PASS "yaml ${f}" || \
        report FAIL "yaml ${f}" "${ERR}" || true
done

XML_FILES=()
while IFS= read -r f; do
    XML_FILES+=("$f")
done < <(
    {
        ls src/navigation/path_generator/mppi/package.xml 2>/dev/null
        ls src/navigation/path_generator/waypoint_publisher/package.xml 2>/dev/null
        ls src/detection/task2_perception/package.xml 2>/dev/null
        find src -maxdepth 4 -path '*task2*' -name 'package.xml' 2>/dev/null
        ls src/robot/config/navigate_through_poses_w_replanning_and_recovery.xml 2>/dev/null
    } | sort -u
)

for f in "${XML_FILES[@]}"; do
    ERR="$(python3 -c "import xml.etree.ElementTree as ET,sys; ET.parse(sys.argv[1])" "$f" 2>&1)" && \
        report PASS "xml ${f}" || \
        report FAIL "xml ${f}" "${ERR}" || true
done

# ------------------------------------------------------------------
# 3. pytest: perception, mission, and optional robot static checks
# ------------------------------------------------------------------
TASK2_PERCEPTION_DIR="$(find src -maxdepth 4 -type d -name 'task2_perception' 2>/dev/null | head -n1 || true)"
if [ -n "${TASK2_PERCEPTION_DIR}" ] && [ -d "${TASK2_PERCEPTION_DIR}/test" ]; then
    OUT="$(python3 -m pytest "${TASK2_PERCEPTION_DIR}/test" -q 2>&1)" && \
        report PASS "pytest task2_perception" "$(echo "${OUT}" | tail -n1)" || \
        report FAIL "pytest task2_perception" "$(echo "${OUT}" | tail -n1)" || true
else
    echo "(task2_perception tests not present in this checkout - skipping)"
fi

MISSION_TESTS=(
    src/mission/mission_manager/test/test_executors.py
    src/mission/mission_manager/test/test_readiness_evidence.py
    src/mission/mission_manager/test/test_registry_and_waypoints.py
)
MISSION_TESTS_PRESENT=()
for f in "${MISSION_TESTS[@]}"; do
    [ -f "${f}" ] && MISSION_TESTS_PRESENT+=("${f}")
done
if [ "${#MISSION_TESTS_PRESENT[@]}" -gt 0 ]; then
    OUT="$(PYTHONPATH=src/mission/mission_manager python3 -m pytest "${MISSION_TESTS_PRESENT[@]}" -q 2>&1)" && \
        report PASS "pytest mission Task2 checks" "$(echo "${OUT}" | tail -n1)" || \
        report FAIL "pytest mission Task2 checks" "$(echo "${OUT}" | tail -n1)" || true
fi

if [ -f src/robot/test/test_launch_roles.py ]; then
    OUT="$(python3 -m pytest src/robot/test/test_launch_roles.py -k task2 -q 2>&1)" && \
        report PASS "pytest Task2 robot launch checks" "$(echo "${OUT}" | tail -n1)" || \
        report FAIL "pytest Task2 robot launch checks" "$(echo "${OUT}" | tail -n1)" || true
else
    report FAIL "pytest Task2 robot launch checks" "test_launch_roles.py missing"
fi

if [ -d src/sim/task2_sim/test ]; then
    OUT="$(PYTHONPATH=src/sim/task2_sim python3 -m pytest src/sim/task2_sim/test -q 2>&1)" && \
        report PASS "pytest Task2 simulation" "$(echo "${OUT}" | tail -n1)" || \
        report FAIL "pytest Task2 simulation" "$(echo "${OUT}" | tail -n1)" || true
fi

# ------------------------------------------------------------------
# Summary
# ------------------------------------------------------------------
echo
echo "=== Summary: ${PASS_COUNT} PASS / ${FAIL_COUNT} FAIL ==="
for line in "${RESULTS[@]}"; do
    echo "  ${line}"
done

if [ "${FAIL_COUNT}" -gt 0 ]; then
    exit 1
fi

exit 0
