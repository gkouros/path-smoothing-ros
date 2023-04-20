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


add_library(${PROJECT_NAME} src/cubic_spline_interpolator.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(demo src/demo.cpp)
add_executable(path_smoother src/path_smoother.cpp)
add_executable(demo_generator src/demo_generator.cpp)

target_link_libraries(demo ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(path_smoother ${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(demo_generator ${PROJECT_NAME} ${catkin_LIBRARIES})

