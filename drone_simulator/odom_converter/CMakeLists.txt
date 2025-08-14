cmake_minimum_required(VERSION 3.0.2)
project(odom_converter)


set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES odom_converter
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

add_library(odom_converter_lib
  include/odom_converter/odom_converter.cpp
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_converter_node
  src/odom_converter_node.cpp
)

target_link_libraries(odom_converter_node
odom_converter_lib  
${catkin_LIBRARIES}
)