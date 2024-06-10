cmake_minimum_required(VERSION 3.0.2)
project(hello_world)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  roslib
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS    roscpp std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})



add_executable( hello_world src/main.cpp )
target_link_libraries( hello_world ${catkin_LIBRARIES} )
