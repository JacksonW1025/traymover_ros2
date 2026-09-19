#!/usr/bin/env bash
# One owned process group for the complete option-20 stack and its diagnostics.
set -Eeuo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
if [[ $# -ne 2 ]]; then
    echo "Usage: $0 <prepared-map-directory> <run-directory>" >&2
    exit 2
fi
MAP_DIRECTORY="$1"
RUN_DIRECTORY="$2"
mkdir -p "${RUN_DIRECTORY}/ros"
export ROS_LOG_DIR="${RUN_DIRECTORY}/ros"
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

# No hardware has started at this point. Catch missing overlay/builds first.
python3 - "${MAP_DIRECTORY}" <<'PY'
import json
import sys
from pathlib import Path
from ament_index_python.packages import get_package_prefix, get_package_share_directory

root = Path(sys.argv[1])
manifest = json.loads((root / 'manifest.json').read_text())
for filename in ('navigation.yaml', 'navigation.pgm', 'localization.yaml', 'localization.pgm'):
    if not (root / filename).is_file():
        raise SystemExit(f'Missing prepared map: {root / filename}')
for package in ('nav2_amcl', 'nav2_map_server', 'nav2_controller', 'nav2_planner',
                'nav2_behaviors', 'nav2_bt_navigator', 'nav2_lifecycle_manager',
                'realsense2_camera', 'fast_lio', 'traymover_robot_safety'):
    get_package_share_directory(package)
nav = Path(get_package_prefix('traymover_robot_nav'))
for executable in ('option20_odom.py', 'option20_clouds.py', 'option20_depth.py',
                   'option20_guard.py', 'option20_pcd_display.py'):
    import os
    if not os.access(nav / 'lib/traymover_robot_nav' / executable, os.X_OK):
        raise SystemExit('Option 20 is not built. Run: colcon build --packages-select traymover_robot_nav --symlink-install')
print('PCD:', manifest['pcd_path'])
PY

# Do not kill unrelated ROS processes. A second stack would compete for sensors
# and TF. Graph discovery is direct (no potentially stale ROS daemon cache).
python3 - <<'PY'
import time
import rclpy
from rclpy.node import Node
rclpy.init()
node = Node('option20_preflight')
deadline = time.monotonic() + 2.0
while time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=.1)
conflicts = {'turn_on_traymover_robot', 'fastlio_mapping', 'amcl',
             'lidar_localization', 'controller_server', 'option20_guard',
             'traymover_imu_publisher', 'lslidar_driver_node'}
active = sorted(set(node.get_node_names()) & conflicts)
node.destroy_node()
rclpy.shutdown()
if active:
    raise SystemExit('Existing robot stack: ' + ', '.join(active) +
                     '. Stop it with menu option 0 before option 20.')
PY

pids=()
cleanup() {
    trap '' INT TERM HUP
    local pid attempt index
    for index in "${!pids[@]}"; do
        pid="${pids[index]}"
        if [[ "${index}" == 0 ]]; then
            # Launch forwards SIGINT to its nodes; signalling the whole group
            # here would interrupt their shutdown twice.
            kill -INT "${pid}" 2>/dev/null || true
        else
            kill -INT -- "-${pid}" 2>/dev/null || true
        fi
    done
    for attempt in {1..40}; do
        local running=false
        for pid in "${pids[@]}"; do
            if kill -0 -- "-${pid}" 2>/dev/null; then running=true; fi
        done
        [[ "${running}" == true ]] || break
        sleep .1
    done
    for pid in "${pids[@]}"; do
        kill -TERM -- "-${pid}" 2>/dev/null || true
    done
    for attempt in {1..20}; do
        local running=false
        for pid in "${pids[@]}"; do
            if kill -0 -- "-${pid}" 2>/dev/null; then running=true; fi
        done
        [[ "${running}" == true ]] || break
        sleep .1
    done
    for pid in "${pids[@]}"; do
        kill -KILL -- "-${pid}" 2>/dev/null || true
        wait "${pid}" 2>/dev/null || true
    done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

setsid ros2 launch traymover_robot_nav option20.launch.py \
    "map_directory:=${MAP_DIRECTORY}" bringup_hardware:=true launch_rviz:=true \
    > >(tee "${RUN_DIRECTORY}/launch.log") 2>&1 &
pids+=("$!")
nav_pid="$!"
setsid python3 "${SCRIPT_DIR}/fastlio_diagnostics.py" \
    --output-dir "${RUN_DIRECTORY}/fastlio" --use-sim-time false \
    --odom-topic /odom --interval-sec 2 > "${RUN_DIRECTORY}/diagnostics.log" 2>&1 &
pids+=("$!")
setsid ros2 topic echo /option20/health std_msgs/msg/String --no-daemon \
    > "${RUN_DIRECTORY}/health.log" 2>&1 &
pids+=("$!")
echo "Option 20 logs: ${RUN_DIRECTORY}"
echo 'In RViz set 2D Pose Estimate; wait for /option20/health ready_for_goal: true.'
echo 'Then set a short 2D Goal Pose. Ctrl+C stops this complete stack.'
wait "${nav_pid}"
