cmake_minimum_required(VERSION 3.8)
project(traymover_robot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/imu_pose_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
