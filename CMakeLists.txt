cmake_minimum_required(VERSION 2.8.3)
project(ur5_core_pkg)


find_package(catkin REQUIRED COMPONENTS
rospy
  roscpp
  visualization_msgs
  geometry_msgs
  std_msgs
  roslib
  message_generation
 )
 find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)

catkin_package(
 DEPENDS Eigen
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib message_runtime std_msgs
 LIBRARIES ur5_core_pkg
)

 include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp library 
add_library(${PROJECT_NAME} src/UR5Core.cpp include/ur5_core_pkg/UR5Core.h)

add_executable(ur5_main src/ur5_main.cpp)
target_link_libraries(ur5_main ur5_core_pkg ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "--std=c++11 ${CMAKE_CXX_FLAGS}")

