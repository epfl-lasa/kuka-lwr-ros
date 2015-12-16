# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "kuka_fri_bridge: 1 messages, 1 services")

set(MSG_I_FLAGS "-Ikuka_fri_bridge:/home/guillaume/roscode/catkin_ws/src/kuka/kuka_interface_packages/kuka_fri_bridge/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(kuka_fri_bridge_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/guillaume/roscode/catkin_ws/src/kuka/kuka_interface_packages/kuka_fri_bridge/msg/JointStateImpedance.msg" NAME_WE)
add_custom_target(_kuka_fri_bridge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "kuka_fri_bridge" "/home/guillaume/roscode/catkin_ws/src/kuka/kuka_interface_packages/kuka_fri_bridge/msg/JointStateImpedance.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/guillaume/roscode/catkin_ws/src/kuka/kuka_interface_packages/kuka_fri_bridge/srv/StringService.srv" NAME_WE)
add_custom_target(_kuka_fri_bridge_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "kuka_fri_bridge" "/home/guillaume/roscode/catkin_ws/src/kuka/kuka_interface_packages/kuka_fri_bridge/srv/StringService.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(kuka_fri_bridge
  "/home/guillaume/roscode/catkin_ws/src/kuka/kuka_interface_packages/kuka_fri_bridge/msg/JointStateImpedance.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kuka_fri_bridge
)

### Generating Services
_generate_srv_cpp(kuka_fri_bridge
  "/home/guillaume/roscode/catkin_ws/src/kuka/kuka_interface_packages/kuka_fri_bridge/srv/StringService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kuka_fri_bridge
)

### Generating Module File
_generate_module_cpp(kuka_fri_bridge
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kuka_fri_bridge
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(kuka_fri_bridge_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(kuka_fri_bridge_generate_messages kuka_fri_bridge_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/guillaume/roscode/catkin_ws/src/kuka/kuka_interface_packages/kuka_fri_bridge/msg/JointStateImpedance.msg" NAME_WE)
add_dependencies(kuka_fri_bridge_generate_messages_cpp _kuka_fri_bridge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/guillaume/roscode/catkin_ws/src/kuka/kuka_interface_packages/kuka_fri_bridge/srv/StringService.srv" NAME_WE)
add_dependencies(kuka_fri_bridge_generate_messages_cpp _kuka_fri_bridge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kuka_fri_bridge_gencpp)
add_dependencies(kuka_fri_bridge_gencpp kuka_fri_bridge_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kuka_fri_bridge_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(kuka_fri_bridge
  "/home/guillaume/roscode/catkin_ws/src/kuka/kuka_interface_packages/kuka_fri_bridge/msg/JointStateImpedance.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kuka_fri_bridge
)

### Generating Services
_generate_srv_lisp(kuka_fri_bridge
  "/home/guillaume/roscode/catkin_ws/src/kuka/kuka_interface_packages/kuka_fri_bridge/srv/StringService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kuka_fri_bridge
)

### Generating Module File
_generate_module_lisp(kuka_fri_bridge
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kuka_fri_bridge
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(kuka_fri_bridge_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(kuka_fri_bridge_generate_messages kuka_fri_bridge_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/guillaume/roscode/catkin_ws/src/kuka/kuka_interface_packages/kuka_fri_bridge/msg/JointStateImpedance.msg" NAME_WE)
add_dependencies(kuka_fri_bridge_generate_messages_lisp _kuka_fri_bridge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/guillaume/roscode/catkin_ws/src/kuka/kuka_interface_packages/kuka_fri_bridge/srv/StringService.srv" NAME_WE)
add_dependencies(kuka_fri_bridge_generate_messages_lisp _kuka_fri_bridge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kuka_fri_bridge_genlisp)
add_dependencies(kuka_fri_bridge_genlisp kuka_fri_bridge_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kuka_fri_bridge_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(kuka_fri_bridge
  "/home/guillaume/roscode/catkin_ws/src/kuka/kuka_interface_packages/kuka_fri_bridge/msg/JointStateImpedance.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kuka_fri_bridge
)

### Generating Services
_generate_srv_py(kuka_fri_bridge
  "/home/guillaume/roscode/catkin_ws/src/kuka/kuka_interface_packages/kuka_fri_bridge/srv/StringService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kuka_fri_bridge
)

### Generating Module File
_generate_module_py(kuka_fri_bridge
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kuka_fri_bridge
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(kuka_fri_bridge_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(kuka_fri_bridge_generate_messages kuka_fri_bridge_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/guillaume/roscode/catkin_ws/src/kuka/kuka_interface_packages/kuka_fri_bridge/msg/JointStateImpedance.msg" NAME_WE)
add_dependencies(kuka_fri_bridge_generate_messages_py _kuka_fri_bridge_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/guillaume/roscode/catkin_ws/src/kuka/kuka_interface_packages/kuka_fri_bridge/srv/StringService.srv" NAME_WE)
add_dependencies(kuka_fri_bridge_generate_messages_py _kuka_fri_bridge_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(kuka_fri_bridge_genpy)
add_dependencies(kuka_fri_bridge_genpy kuka_fri_bridge_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS kuka_fri_bridge_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kuka_fri_bridge)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/kuka_fri_bridge
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(kuka_fri_bridge_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(kuka_fri_bridge_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kuka_fri_bridge)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/kuka_fri_bridge
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(kuka_fri_bridge_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(kuka_fri_bridge_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kuka_fri_bridge)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kuka_fri_bridge\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/kuka_fri_bridge
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(kuka_fri_bridge_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(kuka_fri_bridge_generate_messages_py geometry_msgs_generate_messages_py)
