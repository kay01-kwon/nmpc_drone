# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "l1_estimator: 1 messages, 0 services")

set(MSG_I_FLAGS "-Il1_estimator:/home/kay/catkin_ws/src/research/l1_estimator/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(l1_estimator_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/kay/catkin_ws/src/research/l1_estimator/msg/Lpf_test.msg" NAME_WE)
add_custom_target(_l1_estimator_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "l1_estimator" "/home/kay/catkin_ws/src/research/l1_estimator/msg/Lpf_test.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(l1_estimator
  "/home/kay/catkin_ws/src/research/l1_estimator/msg/Lpf_test.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/l1_estimator
)

### Generating Services

### Generating Module File
_generate_module_cpp(l1_estimator
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/l1_estimator
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(l1_estimator_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(l1_estimator_generate_messages l1_estimator_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kay/catkin_ws/src/research/l1_estimator/msg/Lpf_test.msg" NAME_WE)
add_dependencies(l1_estimator_generate_messages_cpp _l1_estimator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(l1_estimator_gencpp)
add_dependencies(l1_estimator_gencpp l1_estimator_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS l1_estimator_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(l1_estimator
  "/home/kay/catkin_ws/src/research/l1_estimator/msg/Lpf_test.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/l1_estimator
)

### Generating Services

### Generating Module File
_generate_module_eus(l1_estimator
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/l1_estimator
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(l1_estimator_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(l1_estimator_generate_messages l1_estimator_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kay/catkin_ws/src/research/l1_estimator/msg/Lpf_test.msg" NAME_WE)
add_dependencies(l1_estimator_generate_messages_eus _l1_estimator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(l1_estimator_geneus)
add_dependencies(l1_estimator_geneus l1_estimator_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS l1_estimator_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(l1_estimator
  "/home/kay/catkin_ws/src/research/l1_estimator/msg/Lpf_test.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/l1_estimator
)

### Generating Services

### Generating Module File
_generate_module_lisp(l1_estimator
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/l1_estimator
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(l1_estimator_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(l1_estimator_generate_messages l1_estimator_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kay/catkin_ws/src/research/l1_estimator/msg/Lpf_test.msg" NAME_WE)
add_dependencies(l1_estimator_generate_messages_lisp _l1_estimator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(l1_estimator_genlisp)
add_dependencies(l1_estimator_genlisp l1_estimator_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS l1_estimator_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(l1_estimator
  "/home/kay/catkin_ws/src/research/l1_estimator/msg/Lpf_test.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/l1_estimator
)

### Generating Services

### Generating Module File
_generate_module_nodejs(l1_estimator
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/l1_estimator
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(l1_estimator_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(l1_estimator_generate_messages l1_estimator_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kay/catkin_ws/src/research/l1_estimator/msg/Lpf_test.msg" NAME_WE)
add_dependencies(l1_estimator_generate_messages_nodejs _l1_estimator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(l1_estimator_gennodejs)
add_dependencies(l1_estimator_gennodejs l1_estimator_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS l1_estimator_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(l1_estimator
  "/home/kay/catkin_ws/src/research/l1_estimator/msg/Lpf_test.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/l1_estimator
)

### Generating Services

### Generating Module File
_generate_module_py(l1_estimator
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/l1_estimator
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(l1_estimator_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(l1_estimator_generate_messages l1_estimator_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/kay/catkin_ws/src/research/l1_estimator/msg/Lpf_test.msg" NAME_WE)
add_dependencies(l1_estimator_generate_messages_py _l1_estimator_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(l1_estimator_genpy)
add_dependencies(l1_estimator_genpy l1_estimator_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS l1_estimator_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/l1_estimator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/l1_estimator
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(l1_estimator_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/l1_estimator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/l1_estimator
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(l1_estimator_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/l1_estimator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/l1_estimator
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(l1_estimator_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/l1_estimator)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/l1_estimator
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(l1_estimator_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/l1_estimator)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/l1_estimator\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/l1_estimator
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(l1_estimator_generate_messages_py std_msgs_generate_messages_py)
endif()
