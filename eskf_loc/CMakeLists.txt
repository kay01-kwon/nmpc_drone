cmake_minimum_required(VERSION 3.0.2)
project(eskf_loc)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES eskf_loc
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_library(eskf_lib
  include/eskf_loc/eskf_loc.cpp
  include/eskf_ros/eskf_ros.cpp
)

add_executable(eskf_node
  src/eskf_node.cpp
)

target_link_libraries(eskf_node
  eskf_lib
  ${catkin_LIBRARIES}
)