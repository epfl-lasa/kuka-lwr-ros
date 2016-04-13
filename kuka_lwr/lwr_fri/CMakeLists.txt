cmake_minimum_required(VERSION 2.8.3)
project(lwr_fri)

find_package(catkin REQUIRED
fri_library_ros
lwr_console
message_generation
sensor_msgs
geometry_msgs
lwr_hw
)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -isystem -Wall -Wno-deprecated-declarations -Wno-unused-variable -Wno-reorder")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   JointStates.msg
   FRI.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   StringService.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs 
   geometry_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES robot_publisher
  CATKIN_DEPENDS message_runtime fri_library_ros lwr_console
#  DEPENDS system_lib
)

###########
## Build ##
###########


set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)


include_directories(include ${catkin_INCLUDE_DIRS} ${INCLUDE_DIR})



set(H_FILES ${INCLUDE_DIR}/lwr_fri/LWRRobot_FRI.h
            ${INCLUDE_DIR}/lwr_fri/utilities.h
            ${INCLUDE_DIR}/lwr_fri/lwr_fri_console.h
            ${INCLUDE_DIR}/lwr_fri/lwr_fri_interface.h
)

set(S_FILES  ${SRC_DIR}/lwr_fri/LWRRobot_FRI.cpp
             ${SRC_DIR}/lwr_fri/lwr_fri_interface.cpp
             ${SRC_DIR}/lwr_fri/lwr_fri_console.cpp
)

include_directories(${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} ${H_FILES} ${S_FILES} )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(lwr_fri_node ${SRC_DIR}/lwr_fri_node.cpp )
target_link_libraries(lwr_fri_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(lwr_fri_console_node ${SRC_DIR}/lwr_fri_console_node.cpp)
target_link_libraries(lwr_fri_console_node ${PROJECT_NAME} ${catkin_LIBRARIES})


add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)


 
