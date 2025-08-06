cmake_minimum_required(VERSION 3.0.2)
project(cmd_to_rps_converter)

# add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 14)

set(catkin_Devel_Include_DIRS "~/catkin_ws/devel/include")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cmd_to_rps_converter
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${catkin_Devel_Include_DIRS}
)


add_library(converter_lib
  include/cmd_to_rps_converter/cmd_to_rps_converter.cpp
  include/rps_to_rpm_converter/rps_to_rpm_converter.cpp
)

add_executable(cmd_to_rps_converter_node
  src/cmd_to_rps_converter_node.cpp
)

target_link_libraries(cmd_to_rps_converter_node
  converter_lib
  ${catkin_LIBRARIES}
)

add_executable(rps_to_rpm_converter_node
  src/rps_to_rpm_converter_node.cpp
)
target_link_libraries(rps_to_rpm_converter_node
  converter_lib
  ${catkin_LIBRARIES}
)