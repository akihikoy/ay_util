cmake_minimum_required(VERSION 3.0.2)
project(ay_util_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  moveit_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Float64Stamped.msg
  Int32Stamped.msg
  SimpleRobotState.msg
  StringStamped.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  DxlGMove.srv
  DxlIO.srv
  GetAllowedCollisionMatrix.srv
  GetStateValidity.srv
  GetTrajectoryValidity.srv
  SetFlag.srv
  SetPlanningSceneDiff.srv
  SetPUI.srv
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
  moveit_msgs
)


## The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs moveit_msgs
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


## Install environment is not configured.

## Testing method is not configured.

