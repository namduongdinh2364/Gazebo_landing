cmake_minimum_required(VERSION 2.8.3)
project(getpose)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        geometry_msgs roscpp rospy std_msgs tf2 tf2_ros std_msgs)

catkin_package(
        CATKIN_DEPENDS geometry_msgs std_msgs roscpp rospy std_msgs tf2 tf2_ros)

catkin_install_python(PROGRAMS 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)
file(GLOB SOURCES src/*.cpp)
add_executable(${PROJECT_NAME}_node ${SOURCES} )
add_library(${PROJECT_NAME}_node_plugin SHARED ${SOURCES})
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        )
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
