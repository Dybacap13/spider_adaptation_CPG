cmake_minimum_required(VERSION 3.0.2)
project(reward_learning_gyro)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package( catkin REQUIRED COMPONENTS  roscpp sensor_msgs std_msgs nav_msgs hexapod_msgs gazebo_msgs)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS  roscpp  sensor_msgs std_msgs nav_msgs hexapod_msgs gazebo_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})



add_executable( calculator_reward_gyro src/reward_gyro_node.cpp )
target_link_libraries( calculator_reward_gyro ${catkin_LIBRARIES} )
