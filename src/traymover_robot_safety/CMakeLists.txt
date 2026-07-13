cmake_minimum_required(VERSION 3.8)
project(traymover_robot_safety)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(CMAKE_AUTOMOC ON)

set(HDR_FILES
  include/traymover_robot_safety/estop_panel.hpp
)

qt5_wrap_cpp(MOC_FILES ${HDR_FILES})

add_library(${PROJECT_NAME} SHARED
  src/estop_panel.cpp
  ${HDR_FILES}
  ${MOC_FILES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  std_msgs
  std_srvs
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Widgets
)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

add_library(depth_estop_evaluator STATIC
  src/depth_estop_evaluator.cpp
)
set_target_properties(depth_estop_evaluator PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(depth_estop_evaluator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(depth_estop_evaluator sensor_msgs)

add_executable(traymover_auto_estop
  src/auto_estop_node.cpp
)
target_link_libraries(traymover_auto_estop depth_estop_evaluator)
ament_target_dependencies(traymover_auto_estop
  rclcpp
  sensor_msgs
  std_msgs
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS traymover_auto_estop
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  PROGRAMS scripts/traymover_estop_keyboard
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(
    test_depth_estop_evaluator
    test/test_depth_estop_evaluator.cpp
  )
  target_link_libraries(test_depth_estop_evaluator depth_estop_evaluator)
  ament_target_dependencies(test_depth_estop_evaluator sensor_msgs)
  ament_add_pytest_test(
    test_safety_config
    test/test_safety_config.py
  )
endif()

ament_export_dependencies(
  rclcpp
  rviz_common
  sensor_msgs
  std_msgs
  std_srvs
)
ament_export_libraries(${PROJECT_NAME})

ament_package()
