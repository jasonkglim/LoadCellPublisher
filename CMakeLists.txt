cmake_minimum_required(VERSION 3.0.2)
project(LoadCellPublisher)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

## Declare message files to be built
add_message_files(
  FILES
  LoadCellArray.msg
)

## Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

## Specify include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)
