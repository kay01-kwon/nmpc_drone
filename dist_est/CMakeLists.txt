cmake_minimum_required(VERSION 3.0.2)
project(dist_est)

set(CMAKE_CXX_STANDARD 14)
set(catkin_Devel_Include_DIRS "~/catkin_ws/devel/include")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dist_est
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${catkin_Devel_Include_DIRS}
)

# 1. Add libraries for EKF
add_library(ekf_dist_est_lib
  include/utils/forward_dynamics.cpp
  include/dist_est/ekf_dist_est.cpp
  include/node/ekf_node.cpp
)

# 2. Add libraries for HGDO
add_library(hgdo_lib
  include/utils/forward_dynamics.cpp
  include/model/hgdo_model.cpp
  include/dist_est/hgdo_dist_est.cpp
  include/node/hgdo_node.cpp
)

add_executable(ekf_node
  src/ros_ekf.cpp
)
target_link_libraries(ekf_node
  ekf_dist_est_lib
  ${catkin_LIBRARIES}
)

add_executable(hgdo_node
  src/ros_hgdo.cpp
)

target_link_libraries(hgdo_node
  hgdo_lib
  ${catkin_LIBRARIES}
)

add_executable(sync_example_node
  src/ros_sync_example.cpp
  include/node/sync_node_example.cpp
)

target_link_libraries(sync_example_node
  ${catkin_LIBRARIES}
)