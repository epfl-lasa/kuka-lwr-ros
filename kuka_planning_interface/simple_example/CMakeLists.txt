cmake_minimum_required(VERSION 2.8.3)
project(simple_example)

find_package(catkin REQUIRED kuka_action_server kuka_action_client tf)


set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall -Wno-deprecated-declarations")


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
 # INCLUDE_DIRS include
  LIBRARIES simple_example
  CATKIN_DEPENDS kuka_action_server kuka_action_client
#  DEPENDS system_lib
)

###########
## Build ##
###########

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

include_directories(include ${catkin_INCLUDE_DIRS})

# --- Create Some Simple Actions

add_library(simple_actions ${SRC_DIR}/simple_actions/linear_cart_action.cpp
                           ${INCLUDE_DIR}/simple_actions/linear_cart_action.h)

target_link_libraries(simple_actions ${catkin_LIBRARIES} )
# --- Create Action Server Node

add_executable(server_action_node  ${SRC_DIR}/server_action_node.cpp                )
target_link_libraries(server_action_node simple_actions  ${catkin_LIBRARIES}        )

# --- Create Action Client Node Pour action

add_executable(client_action_node  ${SRC_DIR}/client_action_node.cpp	)
target_link_libraries(client_action_node  simple_actions ${catkin_LIBRARIES})


