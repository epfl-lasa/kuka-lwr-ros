cmake_minimum_required(VERSION 2.8.3)
project(robot_motion_generation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp cmake_modules)

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -Wall -Wno-deprecated-declarations -Wno-unused-variable -Wno-reorder")

find_package(Eigen REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    cmake_modules
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Eigen
)



set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)


include_directories(include ${catkin_INCLUDE_DIRS} ${INCLUDE_DIR})



set(H_FILES ${INCLUDE_DIR}/robot_motion_generation/CDDynamics.h
            ${INCLUDE_DIR}/robot_motion_generation/angular_velocity.h
)

set(S_FILES  ${SRC_DIR}/CDDynamics.cpp
)

include_directories(${catkin_INCLUDE_DIRS})


add_library(${PROJECT_NAME} ${H_FILES} ${S_FILES} )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

