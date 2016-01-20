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
)

#add_definitions(-DUSE_DOUBLE_PRECISION)

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall -Wno-deprecated-declarations")


include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

add_message_files(
   FILES
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
    cfg/rot_stiffness.cfg
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
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  include/lwr_controllers/KinematicChainControllerBase.h
  include/lwr_controllers/one_task_inverse_kinematics.h
  include/lwr_controllers/kuka_joint_state_controller.h
  include/lwr_controllers/joint_position_impedance_controller.h
  include/lwr_controllers/joint_controllers.h
  include/lwr_controllers/joint_kinematics_imp.h
  include/lwr_controllers/PIDKinematicChainControllerBase.h

  #src/joint_impedance_controller.cpp
  src/joint_position_impedance_controller.cpp
  src/kuka_joint_state_controller.cpp


  #src/inverse_dynamics_controller.cpp
  #src/computed_torque_controller.cpp
  src/one_task_inverse_kinematics.cpp
  src/joint_kinematics_imp.cpp
  src/joint_controllers.cpp
  #src/multi_task_priority_inverse_kinematics.cpp
  #src/multi_task_priority_inverse_dynamics.cpp
  #src/minimum_effort_inverse_dynamics.cpp
  #src/backstepping_controller.cpp
  #src/dynamic_sliding_mode_controller.cpp
  #src/dynamic_sliding_mode_controller_task_space.cpp
  #src/one_task_inverse_dynamics_jl.cpp
  #src/gravity_compensation.cpp
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

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
