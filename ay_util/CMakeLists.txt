cmake_minimum_required(VERSION 3.0.2)
project(ay_util)

## Compile as C++11, supported in ROS Kinetic and newer
# Some packages such as MoveIt! needs to be compiled with C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  ay_util_msgs
  roscpp
  eigen_conversions
  moveit_core
  moveit_ros_planning_interface
)

## Uncomment this if the package has a setup.py. This macro ensures
catkin_python_setup()


## The catkin_package macro generates cmake config files for your package
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ay_util
  CATKIN_DEPENDS ay_util_msgs
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(${PROJECT_NAME}_state_validity_checker_node
  src_ros/state_validity_checker_node.cpp
)
add_dependencies(${PROJECT_NAME}_state_validity_checker_node
  ${ay_util_msgs_EXPORTED_TARGETS}
)

## Rename C++ executable without prefix
set_target_properties(${PROJECT_NAME}_state_validity_checker_node PROPERTIES OUTPUT_NAME state_validity_checker_node PREFIX "")

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_state_validity_checker_node
  ${catkin_LIBRARIES}
)


## Install environment is not configured.

## Testing method is not configured.

