cmake_minimum_required(VERSION 2.8.3)
project(path_smoothing_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES path_smoothing_ros
 CATKIN_DEPENDS geometry_msgs roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(path_smoothing_ros
  src/${PROJECT_NAME}/path_smoothing_ros.cpp
)
