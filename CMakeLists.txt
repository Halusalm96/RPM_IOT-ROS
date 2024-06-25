cmake_minimum_required(VERSION 3.0.2)
project(nodelet_hello_world)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  std_msgs
  sensor_msgs
  rospy
  cv_bridge
)

catkin_package(
  # INCLUDE_DIRS include
  LIBRARIES nodelet_hello_world
  CATKIN_DEPENDS nodelet roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(nodelet_hello_world
  src/hello_world.cpp
)

target_link_libraries(nodelet_hello_world
  ${catkin_LIBRARIES}
)

install(PROGRAMS
  src/yolo_processor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

