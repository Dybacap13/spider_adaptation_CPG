cmake_minimum_required(VERSION 3.0.2)
project(test_learning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hexapod_msgs
  roscpp
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS  roscpp   hexapod_msgs
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)



add_executable( test_tripod_gait src/test_tripod_gait.cpp )
target_link_libraries( test_tripod_gait ${catkin_LIBRARIES} )

add_executable( test_wrong_gait src/test_wrong_gait.cpp )
target_link_libraries( test_wrong_gait ${catkin_LIBRARIES} )
