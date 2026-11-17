cmake_minimum_required(VERSION 3.5)
project(traymover_robot_nav)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
  geometry_msgs
  nav2_core
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcl_interfaces
)

add_library(${PROJECT_NAME}_progress_checker SHARED
  src/yaw_progress_checker.cpp
)
ament_target_dependencies(${PROJECT_NAME}_progress_checker ${dependencies})

add_executable(option18_lifecycle_manager src/option18_lifecycle_manager.cpp)
ament_target_dependencies(option18_lifecycle_manager nav2_lifecycle_manager tf2_ros)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_option21_launch test/test_option21_launch.py)
  ament_add_pytest_test(test_option21_simulation test/test_option21_simulation.py)
  ament_add_pytest_test(test_option21_airport test/test_option21_airport.py)
  ament_add_pytest_test(test_option21_replay test/test_option21_replay.py)
  ament_add_pytest_test(test_static_tf_cleanup test/test_static_tf_cleanup.py)
  foreach(option20_test map sensors guard config)
    ament_add_pytest_test(
      test_option20_${option20_test}
      test/test_option20_${option20_test}.py
    )
  endforeach()
  ament_add_pytest_test(
    test_pcd2pgm
    test/test_pcd2pgm.py
  )
  ament_add_pytest_test(
    test_fastlio_online_map_filter
    test/test_fastlio_online_map_filter.py
  )
  ament_add_pytest_test(
    test_pointcloud_angular_filter
    test/test_pointcloud_angular_filter.py
  )
  ament_add_pytest_test(
    test_pcd_clean
    test/test_pcd_clean.py
  )
  ament_add_pytest_test(
    test_nav_simple_config
    test/test_nav_simple_config.py
  )
  ament_add_pytest_test(
    test_nav_clearance_config
    test/test_nav_clearance_config.py
  )
  ament_add_pytest_test(
    test_nav_clearance_planning
    test/test_nav_clearance_planning.py
    TIMEOUT 120
  )
  ament_add_pytest_test(
    test_ndt_replay_tools
    test/test_ndt_replay_tools.py
  )
  ament_add_pytest_test(
    test_motion_diagnostics
    test/test_motion_diagnostics.py
  )
  ament_add_pytest_test(
    test_option10_logging
    test/test_option10_logging.py
  )
  ament_add_pytest_test(
    test_option18_menu
    test/test_option18_menu.py
  )
  ament_add_pytest_test(
    test_option18_launch_graph
    test/test_option18_launch_graph.py
  )
  ament_add_pytest_test(
    test_option18_action_endpoints
    test/test_option18_action_endpoints.py
  )
  ament_add_pytest_test(
    test_option18_navigation_smoke
    test/test_option18_navigation_smoke.py
    TIMEOUT 90
  )
  ament_add_pytest_test(
    test_option18_lifecycle_manager
    test/test_option18_lifecycle_manager.py
    TIMEOUT 90
  )
  ament_add_pytest_test(test_simple_detour test/test_simple_detour.py)
  ament_add_pytest_test(test_detour_forward_progress test/test_detour_forward_progress.py)
  ament_add_pytest_test(
    test_option18_depth_policy
    test/test_option18_depth_policy.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_ndt_height_launch
    test/test_ndt_height_launch.py
  )
  ament_add_pytest_test(
    test_ndt_auto_correction_launch
    test/test_ndt_auto_correction_launch.py
  )
  foreach(detour_test gate core config supervisor goal_proxy velocity_chain)
    ament_add_pytest_test(
      test_detour_${detour_test}
      test/test_detour_${detour_test}.py
    )
  endforeach()
  foreach(guided_test config node test)
    ament_add_pytest_test(
      test_guided_detour_${guided_test}
      test/test_guided_detour_${guided_test}.py
    )
  endforeach()
endif()

install(
  DIRECTORY launch config map rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(
  TARGETS ${PROJECT_NAME}_progress_checker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS option18_lifecycle_manager DESTINATION lib/${PROJECT_NAME})

ament_export_libraries(${PROJECT_NAME}_progress_checker)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

ament_package()
