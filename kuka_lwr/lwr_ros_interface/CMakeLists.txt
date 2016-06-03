cmake_minimum_required(VERSION 2.8.3)
project(lwr_ros_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
roscpp
cmake_modules
genmsg
actionlib_msgs
message_runtime
message_generation
std_msgs
geometry_msgs
trajectory_msgs
actionlib
lwr_controllers
robot_motion_generation
lwr_fri)
#-std=c++14
set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -isystem -fext-numeric-literals -mtune=native -fPIC -Wall -Wno-deprecated-declarations")


generate_messages(DEPENDENCIES
std_msgs
geometry_msgs
lwr_fri
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS tf lwr_controllers message_runtime std_msgs trajectory_msgs lwr_fri
#  DEPENDS system_lib
)


include_directories(include ${catkin_INCLUDE_DIRS})

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)


find_package(Eigen)
include_directories(${INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})


set(H_FILES  	${INCLUDE_DIR}/lwr_ros_interface/switch_controller.h
                ${INCLUDE_DIR}/lwr_ros_interface/ros_ee.h
                ${INCLUDE_DIR}/lwr_ros_interface/ros_joint.h
                ${INCLUDE_DIR}/lwr_ros_interface/ros_ee_j.h
                ${INCLUDE_DIR}/lwr_ros_interface/ros_passive_ds.h
#                ${INCLUDE_DIR}/lwr_ros_interface/lwr_ros_utilities.h
)

set(SRC_FILES   ${SRC_DIR}/ros_joint.cpp
                ${SRC_DIR}/ros_ee_j.cpp
                ${SRC_DIR}/ros_passive_ds.cpp
)

add_library(${PROJECT_NAME} SHARED ${H_FILES} ${SRC_FILES})
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)


