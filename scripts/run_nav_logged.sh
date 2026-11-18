#!/usr/bin/env bash
# Run a navigation command with passive live NDT/FAST-LIO diagnostics.
# ROS and workspace setup are sourced by traymover.sh's terminal wrapper.
set -Eeuo pipefail

if [[ $# -lt 2 ]]; then
    echo "Usage: $0 <run-directory> <navigation-command> [arguments...]" >&2
    exit 2
fi
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_DIR="$1"
shift
ODOMETRY_SOURCE="${TRAYMOVER_ODOMETRY_SOURCE:-fastlio}"
mkdir -p "${RUN_DIR}"/{ndt,fastlio,processes,parameters,ros}
export ROS_LOG_DIR="${RUN_DIR}/ros"
command -v setsid >/dev/null

pids=()
stop_process() {
    local pid="$1" attempt
    kill -INT -- "-${pid}" 2>/dev/null || true
    for attempt in {1..30}; do
        kill -0 -- "-${pid}" 2>/dev/null || break
        sleep 0.1
    done
    if kill -0 -- "-${pid}" 2>/dev/null; then
        kill -TERM -- "-${pid}" 2>/dev/null || true
        for attempt in {1..20}; do
            kill -0 -- "-${pid}" 2>/dev/null || break
            sleep 0.1
        done
        kill -KILL -- "-${pid}" 2>/dev/null || true
    fi
    wait "${pid}" 2>/dev/null || true
}
cleanup() {
    local pid idx
    trap '' INT TERM HUP
    # Navigation was appended last: stop it first, while monitors still log.
    for ((idx=${#pids[@]}-1; idx>=0; idx--)); do
        pid="${pids[idx]}"
        stop_process "${pid}"
    done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

setsid python3 "${SCRIPT_DIR}/ndt_diagnostics.py" \
    --output-dir "${RUN_DIR}/ndt" --use-sim-time false \
    --cloud-topic /point_cloud_localization \
    --interval-sec "${TRAYMOVER_NDT_LOG_INTERVAL_SEC:-5}" \
    > "${RUN_DIR}/processes/ndt_diagnostics.log" 2>&1 &
pids+=("$!")
setsid python3 "${SCRIPT_DIR}/fastlio_diagnostics.py" \
    --output-dir "${RUN_DIR}/fastlio" --use-sim-time false \
    --odom-topic /odom \
    --odometry-source "${ODOMETRY_SOURCE}" \
    --interval-sec "${TRAYMOVER_FASTLIO_LOG_INTERVAL_SEC:-2}" \
    > "${RUN_DIR}/processes/fastlio_diagnostics.log" 2>&1 &
pids+=("$!")

# Wait for subscription setup before starting navigation, so bootstrap logs
# are captured. An import/setup error must be visible before hardware navigation.
for attempt in {1..50}; do
    for pid in "${pids[@]}"; do
        if ! kill -0 "${pid}" 2>/dev/null; then
            echo "Diagnostics failed to start; inspect ${RUN_DIR}/processes" >&2
            exit 1
        fi
    done
    if [[ -s "${RUN_DIR}/ndt/ndt_events.log" && -s "${RUN_DIR}/fastlio/fastlio_events.log" ]]; then
        break
    fi
    sleep 0.1
done
if [[ ! -s "${RUN_DIR}/ndt/ndt_events.log" || ! -s "${RUN_DIR}/fastlio/fastlio_events.log" ]]; then
    echo "Diagnostics startup timed out; inspect ${RUN_DIR}/processes" >&2
    exit 1
fi

echo "[traymover] Live NDT/FAST-LIO diagnostics: ${RUN_DIR}"
setsid "$@" > >(tee "${RUN_DIR}/processes/nav_stack.log") 2>&1 &
nav_pid=$!
pids+=("${nav_pid}")

# Retry while launch creates the nodes. Store failures separately and never
# change node parameters. A failed snapshot does not stop navigation.
pending=(lidar_localization fastlio_mapping pointcloud_nav_preprocessor)
if [[ "${ODOMETRY_SOURCE}" = stm32 ]]; then
    pending=(lidar_localization turn_on_traymover_robot pointcloud_nav_preprocessor)
fi
for attempt in {1..8}; do
    remaining=()
    for node in "${pending[@]}"; do
        kill -0 "${nav_pid}" 2>/dev/null || break 2
        if timeout 2 ros2 param dump "/${node}" --no-daemon \
            > "${RUN_DIR}/parameters/${node}.tmp" \
            2> "${RUN_DIR}/parameters/${node}.stderr"; then
            mv "${RUN_DIR}/parameters/${node}.tmp" "${RUN_DIR}/parameters/${node}.yaml"
        else
            remaining+=("${node}")
        fi
    done
    pending=("${remaining[@]}")
    [[ ${#pending[@]} -gt 0 ]] || break
    sleep 1
done
for node in "${pending[@]}"; do
    echo "[traymover] Parameter snapshot unavailable: ${node}; see parameters/${node}.stderr"
done
for pid in "${pids[@]:0:2}"; do
    if ! kill -0 "${pid}" 2>/dev/null; then
        echo "[traymover] Diagnostic collector exited; inspect ${RUN_DIR}/processes" >&2
    fi
done
wait "${nav_pid}"
