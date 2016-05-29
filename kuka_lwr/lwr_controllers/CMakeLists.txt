cmake_minimum_required(VERSION 2.8.3)
project(lwr_controllers)

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  control_msgs
  forward_command_controller
  control_toolbox
  realtime_tools
  urdf
  roscpp
  kdl_parser
  message_generation
  cmake_modules
  robot_motion_generation
  tf
  dynamic_reconfigure
  passive_ds_control
  eigen_conversions
  lwr_hw
)

#add_definitions(-DUSE_DOUBLE_PRECISION)

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall -Wno-deprecated-declarations")


include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

add_message_files(
   FILES
   FF_FB_plan.msg
   PoseRPY.msg
   RPY.msg
   MultiPriorityTask.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
    cfg/damping_param.cfg
    cfg/damping_param_all.cfg
    cfg/stiffness_param.cfg
    cfg/stiffness_param_all.cfg
    cfg/passive_ds_param.cfg
    cfg/PID.cfg
)

catkin_package(
  CATKIN_DEPENDS
    controller_interface
    control_msgs
    control_toolbox
    realtime_tools
    urdf
    forward_command_controller
    kdl_parser
    message_runtime
    robot_motion_generation
    passive_ds_control
    lwr_hw
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)


set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

include_directories(${INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${INCLUDE_STATISTICS})


set(INC_FILES ${INCLUDE_DIR}/controllers/gravity_compensation.h
              ${INCLUDE_DIR}/controllers/base_controllers.h
              ${INCLUDE_DIR}/controllers/joint_position.h
              ${INCLUDE_DIR}/controllers/cartesian_velocity.h
              ${INCLUDE_DIR}/controllers/ff_fb_cartesian.h
              ${INCLUDE_DIR}/controllers/cartesian_position.h
              ${INCLUDE_DIR}/controllers/change_ctrl_mode.h
              ${INCLUDE_DIR}/controllers/passive_ds.h
              )

set(SRC_FILES ${SRC_DIR}/controllers/gravity_compensation.cpp
              ${SRC_DIR}/controllers/joint_position.cpp
              ${SRC_DIR}/controllers/cartesian_position.cpp
              ${SRC_DIR}/controllers/cartesian_velocity.cpp
              ${SRC_DIR}/controllers/ff_fb_cartesian.cpp
              ${SRC_DIR}/controllers/change_ctrl_mode.cpp
              ${SRC_DIR}/controllers/passive_ds.cpp
)

set(INC_FILES_UTIL
    ${INCLUDE_DIR}/utils/definitions.h
    ${INCLUDE_DIR}/utils/pseudo_inversion.h
    ${INCLUDE_DIR}/utils/skew_symmetric.h
    ${INCLUDE_DIR}/utils/base_safety.h
    ${INCLUDE_DIR}/utils/safety.h
    ${INCLUDE_DIR}/utils/speed_safety.h
    ${INCLUDE_DIR}/utils/contact_safety.h
)

set(SRC_FILES_UTIL
    ${SRC_DIR}/utils/safety.cpp
    ${SRC_DIR}/utils/speed_safety.cpp
    ${SRC_DIR}/utils/contact_safety.cpp
)

add_library(${PROJECT_NAME}
  include/lwr_controllers/KinematicChainControllerBase.h
  include/lwr_controllers/one_task_inverse_kinematics.h
  include/lwr_controllers/kuka_joint_state_controller.h
 # include/lwr_controllers/joint_position_impedance_controller.h
  include/lwr_controllers/joint_controllers.h
  #include/lwr_controllers/joint_kinematics_imp.h
  include/lwr_controllers/PIDKinematicChainControllerBase.h
  #src/joint_position_impedance_controller.cpp
  src/kuka_joint_state_controller.cpp
  src/one_task_inverse_kinematics.cpp
  #src/joint_kinematics_imp.cpp
  src/joint_controllers.cpp
  ${INC_FILES_UTIL}
  ${SRC_FILES_UTIL}
  ${INC_FILES}
  ${SRC_FILES}
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES lwr_controllers_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
