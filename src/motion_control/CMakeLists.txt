cmake_minimum_required(VERSION 3.0.2)
project(motion_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg
  geometry_msgs
  dobot
  camera
  move_base_msgs
  actionlib_msgs
  actionlib
  tf
  nav_msgs
)

add_message_files(
  FILES
)

add_service_files(
  FILES
  getFrontDist.srv
  getRightDist.srv
  getLeftDist.srv
  getRightFrontDist.srv
  getLeftFrontDist.srv
  getDiffAngle.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs dobot camera)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/basic_move.py
  scripts/lidar.py
  scripts/pid.py
  scripts/main_process.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)