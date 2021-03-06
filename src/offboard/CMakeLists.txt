cmake_minimum_required(VERSION 3.0.2)
project(offboard)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES offboard
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# catkin_add_nosetests(test)
# define HILT with mode simulation hardware
add_definitions(-DSITL -DBAR=1)

# Find Gazebo
include_directories( src ../../pid)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

file(GLOB SOURCES "src/*.cpp" ../../pid/*.cpp)
add_executable(offb_node ${SOURCES})
add_library(off_node_plugin SHARED src/offb_node.cpp ../../pid/MiniPID.cpp )
target_link_libraries(offb_node ${catkin_LIBRARIES})


# Build our plugin

#target_link_libraries(off_node_plugin ${GAZEBO_LIBRARIES})



