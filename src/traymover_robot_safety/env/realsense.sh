# Make a user-local extraction of the ROS RealSense packages available when
# the system driver is absent. System-level apt installs need no extra paths.
_traymover_realsense_prefix="${XDG_DATA_HOME:-$HOME/.local/share}/traymover/realsense/opt/ros/${ROS_DISTRO:-humble}"
_traymover_realsense_index="share/ament_index/resource_index/packages/realsense2_camera"
if [ -f "$_traymover_realsense_prefix/$_traymover_realsense_index" ] &&
   [ ! -f "/opt/ros/${ROS_DISTRO:-humble}/$_traymover_realsense_index" ]; then
  ament_prepend_unique_value AMENT_PREFIX_PATH "$_traymover_realsense_prefix"
  ament_prepend_unique_value LD_LIBRARY_PATH "$_traymover_realsense_prefix/lib"
  for _traymover_realsense_lib in "$_traymover_realsense_prefix"/lib/*-linux-gnu; do
    if [ -d "$_traymover_realsense_lib" ]; then
      ament_prepend_unique_value LD_LIBRARY_PATH "$_traymover_realsense_lib"
    fi
  done
  for _traymover_realsense_python in "$_traymover_realsense_prefix"/local/lib/python*/dist-packages; do
    if [ -d "$_traymover_realsense_python" ]; then
      ament_prepend_unique_value PYTHONPATH "$_traymover_realsense_python"
    fi
  done
fi
unset _traymover_realsense_prefix _traymover_realsense_index
unset _traymover_realsense_lib _traymover_realsense_python
