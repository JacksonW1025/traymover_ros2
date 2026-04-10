cmake_minimum_required(VERSION 3.8)
project(traymover_nav2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch map param rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
