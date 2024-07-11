cmake_minimum_required(VERSION 3.0.2)
project(l1_estimator)

add_compile_options(-std=c++14)

find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Lpf_test.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

catkin_package(
  LIBRARIES l1_estimator
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/l1_estimator_node.cpp)

add_executable(noise_generator_node 
src/noise_generator_node.cpp
include/noise_generator/noise_generator.cpp
include/noise_generator/ros_noise_generator.cpp)

target_link_libraries(noise_generator_node
${catkin_LIBRARIES})

add_executable(lpf_test_node
src/lpf_test_node.cpp
include/l1_estimator/ros_lpf_node.cpp
include/l1_estimator/low_pass_filter.cpp)

target_link_libraries(lpf_test_node
${catkin_LIBRARIES})

add_executable(test_estimator
src/test.cpp
include/l1_estimator/convex_fn.cpp
include/l1_estimator/gamma_proj.cpp
include/l1_estimator/low_pass_filter.cpp
include/l1_estimator/ref_model.cpp
include/l1_estimator/tools.cpp
include/l1_estimator/disturbance_estimator.cpp
include/yaml_converter/read_config.cpp
)

target_link_libraries(test_estimator
${YAML_CPP_LIBRARIES}
${catkin_LIBRARIES}
)