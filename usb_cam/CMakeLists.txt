cmake_minimum_required(VERSION 3.0.2)
project(usb_cam)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES usb_cam
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LIBUSB_INCLUDE_DIRS}
)

add_library(usb_cam
  src/usb_cam.cpp
)

target_link_libraries(usb_cam
  ${catkin_LIBRARIES}
  ${LIBUSB_LIBRARIES}
)

