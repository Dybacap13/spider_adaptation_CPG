cmake_minimum_required(VERSION 3.0.2)
project(move_feet_adaptation)

find_package( catkin REQUIRED COMPONENTS  roscpp geometry_msgs std_msgs hexapod_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS  roscpp geometry_msgs  std_msgs hexapod_msgs
)


set( SOURCES
    src/main_node.cpp
    src/move_feet.cpp
    )
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} ${SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


