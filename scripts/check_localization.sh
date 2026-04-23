#!/bin/bash
# Quick health check for the re-architected lidar_localization after
# starting option 10. Run this in a second terminal once you've clicked
# /initialpose in RViz and the robot is sitting still for ~10 s.
#
# Expected readings (new architecture):
#   /tf map→odom rate ≈ LiDAR scan rate (~10 Hz). Each frame rebroadcasts
#       the last good pose, so this proves TF never ages out.
#   /pcl_pose rate ≈ 1 / ndt_align_interval_s (~0.2 Hz with 5 s interval).
#       /pcl_pose is only published when NDT actually re-aligns.
#   /odom rate ≈ 20 Hz (FAST_LIO only; single publisher).
#   map→odom translation drift at rest < 0.05 m per minute.

set -u
source /opt/ros/humble/setup.bash
source /home/wheeltec/traymover_ros2/install/setup.bash

echo "=== lifecycle ==="
ros2 lifecycle get /lidar_localization 2>&1

echo
echo "=== /odom publishers (must be exactly 1) ==="
ros2 topic info /odom -v 2>&1 | sed -n '1,12p'

echo
echo "=== topic rates (6 s window) ==="
(timeout 6 ros2 topic hz /point_cloud_localization > /tmp/chk_loc_cloud.log 2>&1 &)
(timeout 6 ros2 topic hz /scan                  > /tmp/chk_scan.log 2>&1 &)
(timeout 6 ros2 topic hz /odom             > /tmp/chk_odom.log 2>&1 &)
(timeout 6 ros2 topic hz /pcl_pose         > /tmp/chk_pose.log 2>&1 &)
sleep 7
printf '  /point_cloud_localization : '; tail -2 /tmp/chk_loc_cloud.log | head -1 | sed 's/^ *//'
printf '  /scan                     : '; tail -2 /tmp/chk_scan.log | head -1 | sed 's/^ *//'
printf '  /odom (FAST_LIO) : '; tail -2 /tmp/chk_odom.log | head -1 | sed 's/^ *//'
printf '  /pcl_pose (NDT)  : '; tail -2 /tmp/chk_pose.log | head -1 | sed 's/^ *//'

echo
echo "=== map→odom snapshot #1 ==="
(timeout 3 ros2 run tf2_ros tf2_echo map odom > /tmp/chk_tf1.log 2>&1 &)
sleep 4
grep -A4 "^At time" /tmp/chk_tf1.log | head -6

echo
echo "waiting 15 s at rest..."
sleep 15

echo "=== map→odom snapshot #2 (should differ by <0.05 m if FAST_LIO is stable) ==="
(timeout 3 ros2 run tf2_ros tf2_echo map odom > /tmp/chk_tf2.log 2>&1 &)
sleep 4
grep -A4 "^At time" /tmp/chk_tf2.log | head -6

echo
echo "=== recent lidar_localization warnings ==="
LATEST=$(ls -1td /home/wheeltec/.ros/log/*/ 2>/dev/null | head -1)
if [ -n "$LATEST" ]; then
  grep -iE "NDT|Fitness|Planar lock|jump|didn't converge" "$LATEST"launch.log 2>/dev/null | tail -10
fi
