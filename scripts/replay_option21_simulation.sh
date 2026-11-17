#!/usr/bin/env bash
# Replay observation topics only, in an isolated ROS domain. No chassis is started.
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
RESULTS="${1:-${WORKSPACE_DIR}/logs/option21_simulation/validation}"
SCENARIO="${2:-bias}"
RVIZ_CONFIG="${SCRIPT_DIR}/rviz/option21_simulation.rviz"
if [[ "${SCENARIO}" =~ ^(static|changed|stress)_s[0-9]+_(wheel|fastlio)$ ]]; then
    RVIZ_CONFIG="${SCRIPT_DIR}/rviz/option21_airport.rviz"
else
    case "${SCENARIO}" in
        ideal|bias|dropout) ;;
        *)
            echo "Invalid experiment scenario: ${SCENARIO}" >&2
            echo 'Example: changed_s101_wheel (wheel ends with l)' >&2
            exit 2
            ;;
    esac
fi
if [[ ! -f "${RESULTS}/${SCENARIO}/bag/metadata.yaml" ]]; then
    echo "Missing experiment bag: ${RESULTS}/${SCENARIO}/bag" >&2
    exit 1
fi
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${TRAYMOVER_SIM_REPLAY_DOMAIN:-168}"
export ROS_LOCALHOST_ONLY=1
replay_log_dir="${WORKSPACE_DIR}/logs/option21_replay/$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "${replay_log_dir}"
rviz_log="${replay_log_dir}/rviz.log"
ready_dir="$(mktemp -d)"
rviz_pid=''
static_pid=''
player_pid=''
cleanup() {
    trap - EXIT INT TERM
    for pid in "${player_pid}" "${static_pid}" "${rviz_pid}"; do
        if [[ -n "${pid}" ]]; then kill -TERM "${pid}" 2>/dev/null || true; fi
    done
    for pid in "${player_pid}" "${static_pid}" "${rviz_pid}"; do
        if [[ -n "${pid}" ]]; then wait "${pid}" 2>/dev/null || true; fi
    done
    rm -rf "${ready_dir}"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
echo "Replay: ${SCENARIO}; ROS domain ${ROS_DOMAIN_ID}; waiting for RViz subscriptions..."
echo "RViz log: ${rviz_log}"
rviz2 -d "${RVIZ_CONFIG}" \
    --ros-args -p use_sim_time:=true > "${rviz_log}" 2>&1 &
rviz_pid=$!
python3 "${SCRIPT_DIR}/option21_replay_static.py" "${RESULTS}/${SCENARIO}/bag" \
    --rviz-pid "${rviz_pid}" --ready-file "${ready_dir}/ready" &
static_pid=$!
until [[ -f "${ready_dir}/ready" ]]; do
    if ! kill -0 "${rviz_pid}" 2>/dev/null || ! kill -0 "${static_pid}" 2>/dev/null; then
        echo 'RViz failed to become ready; playback has not started.' >&2
        tail -n 20 "${rviz_log}" >&2
        exit 1
    fi
    sleep .1
done
# /cmd_vel is deliberately excluded even though it is retained in the evidence bag.
# Static topics are supplied by the helper, including old bags with wall-time stamps.
ros2 bag play "${RESULTS}/${SCENARIO}/bag" --clock --rate 0.75 --loop \
    --qos-profile-overrides-path "${RESULTS}/playback_qos.yaml" \
    --topics /tf /odom /pcl_pose /point_cloud_localization \
    /simulation/truth_path /simulation/wheel_path \
    /simulation/localized_path /simulation/goals &
player_pid=$!
# Closing RViz also stops its player and static publisher, and vice versa.
status=0
wait -n "${rviz_pid}" "${static_pid}" "${player_pid}" || status=$?
if (( status != 0 )); then
    echo "Replay process exited with status ${status}. RViz log: ${rviz_log}" >&2
    tail -n 20 "${rviz_log}" >&2
fi
exit "${status}"
