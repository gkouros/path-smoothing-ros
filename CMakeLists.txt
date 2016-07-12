cmake_minimum_required(VERSION 2.8.3)
project(path_smoothing_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
)

catkin_package(
 INCLUDE_DIRS
  include
 CATKIN_DEPENDS
  roscpp
  geometry_msgs
  nav_msgs
 LIBRARIES
  path_smoothing_ros
)

include_directories(include ${catkin_INCLUDE_DIRS})


add_library(cubic_spline_interpolator src/cubic_spline_interpolator.cpp)
target_link_libraries(cubic_spline_interpolator ${catkin_LIBRARIES})

add_executable(demo src/demo.cpp)
target_link_libraries(demo cubic_spline_interpolator ${catkin_LIBRARIES})
