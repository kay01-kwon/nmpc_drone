cmake_minimum_required(VERSION 3.0.2)
project(opt_based_estimator)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

find_package(Python3 3.8 EXACT COMPONENTS Interpreter Development)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES opt_based_estimator
 CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_dist_est 
src/test_dist_est.cpp
include/utils/quaternion_math_tools.cpp
include/utils/plot_tools.cpp
include/simulation_model/rotational_simulation_class.cpp
# include/opt_based_estimator/rotational_disturbance_estimator.cpp
# include/opt_based_estimator/rotational_rk4_gradient.cpp
)

target_include_directories(test_dist_est
PRIVATE
${Python3_INCLUDE_DIRS}
)

target_link_libraries(test_dist_est
# ${YAML_CPP_LIBRARIES}
${catkin_LIBRARIES}
${Python3_LIBRARIES} 
)