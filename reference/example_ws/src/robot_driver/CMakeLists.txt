cmake_minimum_required(VERSION 3.0.2)
project(robot_driver)
########################################
#verstion:
#1.0.0 2020-12-18 ���ӳ�������������
########################################

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  map_msgs
  tf
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES 
  CATKIN_DEPENDS roscpp std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(include ${catkin_INCLUDE_DIRS})

# building c++ executables and libraries
add_library(robot_driver_lib src/rc_helper.cpp)
target_link_libraries(robot_driver_lib ${catkin_LIBRARIES})

add_executable(robot_driver src/driver_ros.cpp)
add_dependencies(robot_driver robot_driver_lib)
target_link_libraries(robot_driver robot_driver_lib)

#############
## Install ##
#############

install(TARGETS robot_driver_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS robot_driver
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch driver_param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
