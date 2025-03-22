cmake_minimum_required(VERSION 3.0.2)
project(wheel_odom)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  move_base_msgs
)

## Find OpenCV library
find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable and add source files
add_executable(${PROJECT_NAME}_node
  src/wheelOdom.cpp
  src/CAN.cpp
)

## Link the executable with catkin libraries
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)


