cmake_minimum_required(VERSION 2.8.3)
project(lwr_mpc_examples)

#set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp
cmake_modules
message_runtime
tf
lwr_controllers
robot_motion_generation
lwr_fri)

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -Wall -Wno-deprecated-declarations -Wno-unused-variable -Wno-reorder")

find_package(Eigen REQUIRED)

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
#  INCLUDE_DIRS include
  LIBRARIES lwr_mpc_examples
  CATKIN_DEPENDS tf lwr_controllers message_runtime lwr_fri robot_motion_generation
  DEPENDS Eigen
)

###########
## Build ##
###########

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})


set(H_FILES ${INCLUDE_DIR}/PointMassLinear.h
)

set(S_FILES  ${SRC_DIR}/lwr_mpc_linear_node.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(lwr_mpc_examples ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(lwr_mpc_linear_node ${S_FILES} ${H_FILES})
target_link_libraries(lwr_mpc_linear_node  ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(lwr_mpc_examples_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(lwr_mpc_examples_node
#   ${catkin_LIBRARIES}
# )
