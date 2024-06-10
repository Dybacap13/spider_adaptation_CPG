cmake_minimum_required(VERSION 3.0.2)
project(reinforcement_learning_shooter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  move_feet

  roscpp
  std_msgs
  roslib
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS  move_feet   roscpp std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})



add_executable( reinforcement_learning src/main.cpp )
target_link_libraries( reinforcement_learning ${catkin_LIBRARIES} )
