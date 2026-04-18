#!/usr/bin/env bash
# Interactive launcher for the Traymover robot.
# Each "start" option opens the command in a new terminal window so the menu
# stays usable and multiple subsystems can run side by side.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_DISTRO_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="${WORKSPACE_DIR}/install/setup.bash"

DEFAULT_MAP_DIR="${WORKSPACE_DIR}/src/traymover_robot_nav2/map"
FASTLIO_PCD_DIR="${WORKSPACE_DIR}/src/traymover_robot_slam/FAST_LIO/PCD"
FASTLIO_ROSBAG_DIR="${WORKSPACE_DIR}/src/traymover_robot_slam/FAST_LIO/rosbag"

# ---- terminal spawning ------------------------------------------------------
# Pick whichever terminal emulator is installed. gnome-terminal on Jetson Orin
# Ubuntu 22.04, xterm as a generic fallback.
pick_terminal() {
    if command -v gnome-terminal >/dev/null 2>&1; then
        echo "gnome-terminal"
    elif command -v xterm >/dev/null 2>&1; then
        echo "xterm"
    else
        echo ""
    fi
}

# Spawn a ROS command in a new terminal window. Args: <window_title> <command...>
spawn_in_terminal() {
    local title="$1"; shift
    local cmd="$*"
    local term
    term="$(pick_terminal)"

    # Wrap the command so the shell sources ROS first and keeps the window open
    # on exit (so the user can read any error before the window closes).
    local wrapped
    wrapped="echo '[traymover] ${title}'; \
source '${ROS_DISTRO_SETUP}'; \
if [ -f '${WS_SETUP}' ]; then source '${WS_SETUP}'; else echo '[traymover] WARNING: ${WS_SETUP} not found — did you colcon build?'; fi; \
${cmd}; \
echo; echo '[traymover] ${title} exited. Press Enter to close.'; read"

    case "${term}" in
        gnome-terminal)
            gnome-terminal --title="${title}" -- bash -lc "${wrapped}" &
            ;;
        xterm)
            xterm -T "${title}" -e bash -lc "${wrapped}" &
            ;;
        *)
            echo "No supported terminal emulator found (gnome-terminal/xterm)."
            echo "Running in this window instead. Ctrl+C to return to the menu."
            bash -lc "${wrapped}"
            return
            ;;
    esac
    # Give the new window a moment to appear before redrawing the menu.
    sleep 0.3
}

# ---- clean slate ------------------------------------------------------------
# Kill anything this launcher may have started previously so each start action
# begins from a known-empty state. We match on launch/run command lines and
# known node executables — broad on purpose (this is a dedicated robot).
KILL_PATTERNS=(
    'ros2 launch turn_on_traymover_robot'
    'ros2 launch traymover_slam_toolbox'
    'ros2 launch traymover_robot_nav2'
    'ros2 launch lslidar_driver'
    'ros2 launch fast_lio'
    'ros2 bag record'
    'ros2 bag play'
    'ros2 run traymover_robot_keyboard'
    'traymover_keyboard'
    'traymover_robot.py'
    'traymover_ekf_filter_node'
    'async_slam_toolbox_node'
    'sync_slam_toolbox_node'
    'lslidar_driver_node'
    'fastlio_mapping'
    'pointcloud_to_laserscan'
    'imu_pose_broadcaster'
    'robot_state_publisher'
    'joint_state_publisher'
    'rviz2'
    'ekf_node'
    '\[traymover\]'  # the wrapper banner, matches bash processes in spawned terminals
)

kill_previous() {
    local quiet="${1:-false}"
    [ "$quiet" = "true" ] || echo "[traymover] Cleaning up previous processes..."
    for p in "${KILL_PATTERNS[@]}"; do
        pkill -f "$p" 2>/dev/null || true
    done
    # Give processes a moment to shut down cleanly, then SIGKILL stragglers.
    sleep 0.8
    for p in "${KILL_PATTERNS[@]}"; do
        pkill -9 -f "$p" 2>/dev/null || true
    done
    [ "$quiet" = "true" ] || echo "[traymover] Cleanup done."
}

# ---- actions ----------------------------------------------------------------

action_start_chassis() {
    # Bringup without lidar/EKF/SLAM — for teleop, IMU, URDF, RViz only.
    spawn_in_terminal "traymover: chassis" \
        "ros2 launch turn_on_traymover_robot turn_on_traymover_robot.launch.py use_lidar:=false use_ekf:=false"
}

action_start_keyboard() {
    spawn_in_terminal "traymover: keyboard" \
        "ros2 run traymover_robot_keyboard traymover_keyboard"
}

action_start_slam() {
    # online_async launch already includes chassis bringup with lidar + EKF.
    spawn_in_terminal "traymover: slam_toolbox" \
        "ros2 launch traymover_slam_toolbox online_async.launch.py"
}

action_save_fastlio_map() {
    # FAST-LIO's /map_save service writes to the compiled-in map_file_path
    # (see config/traymover.yaml). We trigger that, then move the file to a
    # user-chosen name/directory so multiple maps don't overwrite each other.
    local default_name="traymover_$(date +%Y%m%d_%H%M%S)"
    read -r -p "Map name (without .pcd) [default: ${default_name}]: " map_name
    map_name="${map_name:-${default_name}}"
    # Strip a trailing .pcd if the user typed one.
    map_name="${map_name%.pcd}"

    read -r -p "Save directory [default: ${FASTLIO_PCD_DIR}]: " map_dir
    map_dir="${map_dir:-${FASTLIO_PCD_DIR}}"
    mkdir -p "${map_dir}"

    set +u
    # shellcheck disable=SC1090
    source "${ROS_DISTRO_SETUP}"
    if [ -f "${WS_SETUP}" ]; then
        # shellcheck disable=SC1090
        source "${WS_SETUP}"
    fi
    set -u

    local staging="${FASTLIO_PCD_DIR}/traymover.pcd"
    # Remove any stale staging file so we can tell whether this call actually wrote one.
    rm -f "${staging}"

    echo "[traymover] Calling /map_save on FAST-LIO (waiting up to 10s)..."
    local srv_output
    srv_output="$(timeout 10 ros2 service call /map_save std_srvs/srv/Trigger 2>&1)"
    local rc=$?
    if [ $rc -ne 0 ]; then
        echo "[traymover] /map_save call failed. Is FAST-LIO running (option 6)?"
        echo "${srv_output}"
        return 1
    fi

    if [ ! -f "${staging}" ]; then
        echo "[traymover] Service returned but ${staging} was not written."
        echo "[traymover] Likely the accumulated cloud was empty — move the robot and try again."
        echo "${srv_output}"
        return 1
    fi

    local dest="${map_dir%/}/${map_name}.pcd"
    if ! mv "${staging}" "${dest}"; then
        echo "[traymover] Failed to move ${staging} → ${dest}"
        return 1
    fi
    local size
    size="$(du -h "${dest}" | awk '{print $1}')"
    echo "[traymover] Saved ${dest} (${size})"
}

action_start_fastlio() {
    # LiDAR-IMU odometry via FAST-LIO. No STM32 wheel odometry is fed into
    # the estimator — pose comes purely from LSLiDAR CX32 + HiPNUC N300 Pro.
    # The base_serial node is still started so /cmd_vel can drive the chassis
    # (pair this option with option 2 "keyboard" to teleop during mapping).
    # IMU driver is launched standalone here; we pass use_imu:=false to
    # base_serial.launch.py so it does NOT spawn a second IMU publisher.
    spawn_in_terminal "traymover: chassis_serial" \
        "ros2 launch turn_on_traymover_robot base_serial.launch.py use_imu:=false"
    spawn_in_terminal "traymover: imu" \
        "ros2 launch turn_on_traymover_robot traymover_imu.launch.py"
    spawn_in_terminal "traymover: lidar" \
        "ros2 launch lslidar_driver lslidar_cx_launch.py"
    # Give LiDAR/IMU a moment to come up so FAST-LIO's IMU init doesn't stall.
    sleep 5
    spawn_in_terminal "traymover: fast_lio" \
        "ros2 launch fast_lio mapping.launch.py config_file:=traymover.yaml rviz:=true"
}

action_kill_all() {
    kill_previous
}

action_replay_fastlio_bag() {
    # 用之前录的 rosbag 离线跑 FAST-LIO。关键点:fast_lio 和 bag 都走 sim_time
    # (bag play --clock 发布 /clock,fast_lio 用 use_sim_time:=true 订阅它),
    # 这样时间戳匹配、IMU 初始化不会因为 wall clock 差异失败。
    if [ ! -d "${FASTLIO_ROSBAG_DIR}" ] || [ -z "$(ls -A "${FASTLIO_ROSBAG_DIR}" 2>/dev/null)" ]; then
        echo "[traymover] No bags in ${FASTLIO_ROSBAG_DIR}. Record one with option 8 first."
        return 1
    fi

    # 列出候选,按修改时间排序,最新的在最上。
    local -a bags=()
    while IFS= read -r -d '' d; do
        bags+=("$d")
    done < <(find "${FASTLIO_ROSBAG_DIR}" -mindepth 1 -maxdepth 1 -type d -print0 | xargs -0 -I{} stat --format='%Y %n' {} 2>/dev/null | sort -rn | cut -d' ' -f2- | tr '\n' '\0')

    if [ ${#bags[@]} -eq 0 ]; then
        echo "[traymover] No bag directories found in ${FASTLIO_ROSBAG_DIR}."
        return 1
    fi

    echo "Available bags (newest first):"
    local i
    for i in "${!bags[@]}"; do
        printf "  [%d] %s\n" "$((i+1))" "$(basename "${bags[i]}")"
    done

    local pick bag_path
    read -r -p "Select bag number [default: 1 = newest]: " pick
    pick="${pick:-1}"
    if ! [[ "${pick}" =~ ^[0-9]+$ ]] || [ "${pick}" -lt 1 ] || [ "${pick}" -gt "${#bags[@]}" ]; then
        echo "[traymover] Invalid selection: ${pick}"
        return 1
    fi
    bag_path="${bags[$((pick-1))]}"

    echo "[traymover] Replaying bag: ${bag_path}"
    echo "[traymover]   FAST-LIO will run with use_sim_time:=true."
    echo "[traymover]   Select option 7 to save the accumulated PCD afterwards."
    echo "[traymover]   Select option 0 to stop replay + fast_lio."

    # Start FAST-LIO first so it's ready to receive scans once the bag plays.
    spawn_in_terminal "traymover: fast_lio (sim_time)" \
        "ros2 launch fast_lio mapping.launch.py config_file:=traymover.yaml rviz:=true use_sim_time:=true"
    # Let fast_lio subscribe + init before playback starts.
    sleep 5
    spawn_in_terminal "traymover: bag_play" \
        "ros2 bag play '${bag_path}' --clock"
}

action_record_fastlio_bag() {
    # 录制 FAST-LIO 离线建图所需的 topic:LiDAR 点云 + IMU + TF。
    # 传感器拓扑与 action_start_fastlio 对齐,保证 bag 可以直接 replay 给 fast_lio。
    # 启动 rosbag record,0 选项的 pkill -TERM 会让它干净地 flush metadata 再退出。
    mkdir -p "${FASTLIO_ROSBAG_DIR}"
    local ts bag_path
    ts="$(date +%Y%m%d_%H%M%S)"
    bag_path="${FASTLIO_ROSBAG_DIR}/traymover_${ts}"

    echo "[traymover] Starting FAST-LIO sensors + rosbag recording."
    echo "[traymover]   Bag path : ${bag_path}"
    echo "[traymover]   Topics   : /point_cloud_raw /imu/data_raw /tf_static /tf"
    echo "[traymover]   To drive : select option 2 (keyboard) in another pass."
    echo "[traymover]   To stop  : select option 0; bag metadata will be finalized."

    spawn_in_terminal "traymover: chassis_serial" \
        "ros2 launch turn_on_traymover_robot base_serial.launch.py use_imu:=false"
    spawn_in_terminal "traymover: imu" \
        "ros2 launch turn_on_traymover_robot traymover_imu.launch.py"
    spawn_in_terminal "traymover: lidar" \
        "ros2 launch lslidar_driver lslidar_cx_launch.py"
    # Let sensors stabilize so the bag starts with valid data.
    sleep 5
    spawn_in_terminal "traymover: rosbag" \
        "ros2 bag record -o '${bag_path}' /point_cloud_raw /imu/data_raw /tf_static /tf"
}

action_show_battery() {
    # ROS setup.bash 里用了未初始化变量,需要临时关掉 nounset
    set +u
    # shellcheck disable=SC1090
    source "${ROS_DISTRO_SETUP}"
    if [ -f "${WS_SETUP}" ]; then
        # shellcheck disable=SC1090
        source "${WS_SETUP}"
    fi
    set -u

    echo "[traymover] Reading /battery_state (waiting up to 5s)..."
    local output
    output="$(timeout 5 ros2 topic echo /battery_state --once 2>&1)"
    local rc=$?
    if [ $rc -ne 0 ] || ! echo "${output}" | grep -q 'percentage:'; then
        echo "[traymover] Could not read /battery_state. Is the chassis running?"
        echo "[traymover] Start option 1 (chassis) or 3 (SLAM) first, then try again."
        return 1
    fi

    local pct temp charge_status
    pct="$(echo "${output}" | awk '/^percentage:/ {print $2}')"
    temp="$(echo "${output}" | awk '/^temperature:/ {print $2}')"
    charge_status="$(echo "${output}" | awk '/^power_supply_status:/ {print $2}')"

    local pct_display
    if [ -n "${pct}" ]; then
        pct_display="$(awk -v p="${pct}" 'BEGIN{printf "%.0f%%", p*100}')"
    else
        pct_display="?"
    fi

    local charge_text
    case "${charge_status}" in
        1) charge_text="CHARGING" ;;
        2) charge_text="DISCHARGING" ;;
        3) charge_text="NOT_CHARGING" ;;
        4) charge_text="FULL" ;;
        *) charge_text="UNKNOWN(${charge_status})" ;;
    esac

    echo "  Battery : ${pct_display}"
    echo "  Temp    : ${temp:-?} °C"
    echo "  Status  : ${charge_text}"
}

action_save_map() {
    mkdir -p "${DEFAULT_MAP_DIR}"

    local default_name="traymover_$(date +%Y%m%d_%H%M%S)"
    read -r -p "Map name [default: ${default_name}]: " map_name
    map_name="${map_name:-${default_name}}"

    read -r -p "Save directory [default: ${DEFAULT_MAP_DIR}]: " map_dir
    map_dir="${map_dir:-${DEFAULT_MAP_DIR}}"
    mkdir -p "${map_dir}"

    local map_path="${map_dir%/}/${map_name}"
    echo "[traymover] Saving map to ${map_path}.{pgm,yaml} ..."

    # Run in this terminal so the user sees success/failure directly.
    # shellcheck disable=SC1090
    source "${ROS_DISTRO_SETUP}"
    if [ -f "${WS_SETUP}" ]; then
        # shellcheck disable=SC1090
        source "${WS_SETUP}"
    fi

    if ! ros2 run nav2_map_server map_saver_cli -f "${map_path}"; then
        echo "[traymover] map_saver_cli failed. Is slam_toolbox publishing /map?"
        return 1
    fi
    echo "[traymover] Saved ${map_path}.pgm and ${map_path}.yaml"
}

# ---- menu -------------------------------------------------------------------

print_menu() {
    cat <<EOF

===== Traymover Launcher =====
  Workspace: ${WORKSPACE_DIR}
  0) Kill all traymover processes (clean up before starting)
  1) Start chassis           (bringup: description + RViz + IMU + base serial)
  2) Start keyboard teleop
  3) Start SLAM (slam_toolbox async; includes chassis + lidar + EKF)
  4) Save SLAM map
  5) Show battery (requires chassis running)
  6) Start FAST-LIO (LiDAR-IMU odometry + RViz; no STM32 wheel odom)
  7) Save FAST-LIO PCD map (requires option 6 running)
  8) Record FAST-LIO rosbag (sensors + bag; 0 stops and saves)
  9) Replay FAST-LIO rosbag offline (bag + fast_lio with sim_time)
  q) Quit
==============================
EOF
}

main() {
    if [ ! -f "${ROS_DISTRO_SETUP}" ]; then
        echo "ERROR: ROS 2 Humble not found at ${ROS_DISTRO_SETUP}" >&2
        exit 1
    fi

    while true; do
        print_menu
        read -r -p "Select an option: " choice
        case "${choice}" in
            0) action_kill_all ;;
            1) action_start_chassis ;;
            2) action_start_keyboard ;;
            3) action_start_slam ;;
            4) action_save_map ;;
            5) action_show_battery ;;
            6) action_start_fastlio ;;
            7) action_save_fastlio_map ;;
            8) action_record_fastlio_bag ;;
            9) action_replay_fastlio_bag ;;
            q|Q|quit|exit) echo "Bye."; exit 0 ;;
            "") ;;
            *) echo "Unknown option: ${choice}" ;;
        esac
    done
}

main "$@"
