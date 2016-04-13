cmake_minimum_required(VERSION 2.8.3)
project(lwr_ros_client)


find_package(catkin REQUIRED
tf
roscpp
std_msgs
message_runtime
message_generation
lwr_console
lwr_ros_interface
)

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -isystem -mtune=native -fPIC -Wall -Wno-deprecated-declarations -Wno-reorder")

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

add_service_files(FILES
    String_cmd.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp tf std_msgs message_runtime lwr_console lwr_ros_interface
   DEPENDS system_lib
)

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

include_directories(include ${catkin_INCLUDE_DIRS} ${INCLUDE_DIR} )

set(H_FILES     ${INCLUDE_DIR}/lwr_ros_client/kuka_action_client.h
                ${INCLUDE_DIR}/lwr_ros_client/action_client_cmd_interface.h
                ${INCLUDE_DIR}/lwr_ros_client/kuka_action_console.h
                ${INCLUDE_DIR}/lwr_ros_client/ros_param_parser.h)

set(SRC_FILES   ${SRC_DIR}/kuka_action_client.cpp
                ${SRC_DIR}/action_client_cmd_interface.cpp
                ${SRC_DIR}/kuka_action_console.cpp
                ${SRC_DIR}/ros_param_parser.cpp

)

set(H_FILES_ACTION ${INCLUDE_DIR}/lwr_ros_action/joint_action.h
                   ${INCLUDE_DIR}/lwr_ros_action/base_action.h

)
set(SRC_FILES_ACTION ${SRC_DIR}/lwr_ros_action/joint_action.cpp)

add_library(${PROJECT_NAME} ${H_FILES} ${H_FILES_ACTION} ${SRC_FILES} ${SRC_FILES_ACTION})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)

