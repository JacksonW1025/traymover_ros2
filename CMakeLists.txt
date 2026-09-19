cmake_minimum_required(VERSION 3.5)
project(lidar_localization_ros2)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

SET(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs  REQUIRED)
find_package(tf2_sensor_msgs  REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(ndt_omp_ros2 REQUIRED)
find_package(OpenMP)
include(CTest)

if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


add_library(lidar_localization_component SHARED
src/lidar_localization_component.cpp
)

ament_target_dependencies(lidar_localization_component
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_eigen
  geometry_msgs
  sensor_msgs
  nav_msgs
  pcl_conversions
  ndt_omp_ros2
)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS})

add_executable(lidar_localization_node src/lidar_localization_node.cpp)
target_link_libraries(lidar_localization_node
  lidar_localization_component
  ${PCL_LIBRARIES}
  ${rclcpp_lifecycle_LIBRARIES}
  ${std_msgs_LIBRARIES}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

ament_export_libraries(lidar_localization_component)

install(TARGETS
  lidar_localization_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  lidar_localization_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  add_executable(test_weighted_fitness test/test_weighted_fitness.cpp)
  target_link_libraries(test_weighted_fitness ${PCL_LIBRARIES})
  add_test(NAME test_weighted_fitness COMMAND test_weighted_fitness)
  add_executable(test_tf_responsiveness test/test_tf_responsiveness.cpp)
  target_link_libraries(test_tf_responsiveness
    lidar_localization_component ${PCL_LIBRARIES}
    ${rclcpp_lifecycle_LIBRARIES} ${std_msgs_LIBRARIES})
  add_test(NAME test_tf_responsiveness COMMAND test_tf_responsiveness)
  set_tests_properties(test_tf_responsiveness PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=216;ROS_LOCALHOST_ONLY=1" TIMEOUT 30)
  add_executable(test_auto_correction test/test_auto_correction.cpp)
  target_link_libraries(test_auto_correction
    lidar_localization_component ${PCL_LIBRARIES}
    ${rclcpp_lifecycle_LIBRARIES} ${std_msgs_LIBRARIES})
  add_test(NAME test_auto_correction COMMAND test_auto_correction)
  set_tests_properties(test_auto_correction PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=215;ROS_LOCALHOST_ONLY=1")
  add_executable(test_height_weighting test/test_height_weighting.cpp)
  target_link_libraries(test_height_weighting
    lidar_localization_component ${PCL_LIBRARIES}
    ${rclcpp_lifecycle_LIBRARIES} ${std_msgs_LIBRARIES})
  add_test(NAME test_height_weighting COMMAND test_height_weighting)
  set_tests_properties(test_height_weighting PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=214;ROS_LOCALHOST_ONLY=1")
  add_executable(test_ndt_reinitialization test/test_ndt_reinitialization.cpp)
  target_link_libraries(test_ndt_reinitialization
    lidar_localization_component ${PCL_LIBRARIES}
    ${rclcpp_lifecycle_LIBRARIES} ${std_msgs_LIBRARIES})
  add_test(NAME test_ndt_reinitialization COMMAND test_ndt_reinitialization)
  set_tests_properties(test_ndt_reinitialization PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=213;ROS_LOCALHOST_ONLY=1")
  add_executable(test_map_downsampling test/test_map_downsampling.cpp)
  target_link_libraries(test_map_downsampling ${PCL_LIBRARIES})
  add_test(NAME test_map_downsampling COMMAND test_map_downsampling)
  add_executable(test_ndt_map_compatibility
    test/test_ndt_map_compatibility.cpp)
  target_include_directories(test_ndt_map_compatibility PRIVATE include)
  add_test(
    NAME test_ndt_map_compatibility
    COMMAND test_ndt_map_compatibility)
endif()

ament_package()
