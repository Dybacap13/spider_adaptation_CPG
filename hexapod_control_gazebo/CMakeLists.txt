cmake_minimum_required(VERSION 2.8.3)
project( hexapod_control_gazebo )

find_package( catkin REQUIRED COMPONENTS nodelet roscpp geometry_msgs sensor_msgs std_msgs hexapod_msgs tf dynamixel_sdk)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS nodelet roscpp geometry_msgs sensor_msgs std_msgs hexapod_msgs tf
)

include_directories(include ${catkin_INCLUDE_DIRS})



add_executable( hexapod_control_gazebo src/control_gazebo.cpp )
target_link_libraries( hexapod_control_gazebo ${catkin_LIBRARIES} )



install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
