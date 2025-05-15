cmake_minimum_required(VERSION 3.0.2)
project(camera)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg
  geometry_msgs
)

add_message_files(
  FILES
)

add_service_files(
  FILES
  PhotoService.srv
  PhotoboxService.srv
  PhotoshelfService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/take_photo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)