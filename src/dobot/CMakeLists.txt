cmake_minimum_required(VERSION 3.0.2)
project(dobot)

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
  SetCmdTimeout.srv
  GetDeviceSN.srv
  SetDeviceName.srv
  GetDeviceName.srv
  GetDeviceVersion.srv
  GetPose.srv
  GetAlarmsState.srv
  ClearAllAlarmsState.srv
  SetHOMEParams.srv
  GetHOMEParams.srv
  SetHOMECmd.srv
  SetEndEffectorParams.srv
  GetEndEffectorParams.srv
  SetEndEffectorLaser.srv
  GetEndEffectorLaser.srv
  SetEndEffectorSuctionCup.srv
  GetEndEffectorSuctionCup.srv
  SetEndEffectorGripper.srv
  GetEndEffectorGripper.srv
  SetJOGJointParams.srv
  GetJOGJointParams.srv
  SetJOGCoordinateParams.srv
  GetJOGCoordinateParams.srv
  SetJOGCommonParams.srv
  GetJOGCommonParams.srv
  SetJOGCmd.srv
  SetPTPJointParams.srv
  GetPTPJointParams.srv
  SetPTPCoordinateParams.srv
  GetPTPCoordinateParams.srv
  SetPTPJumpParams.srv
  GetPTPJumpParams.srv
  SetPTPCommonParams.srv
  GetPTPCommonParams.srv
  SetPTPCmd.srv
  SetCPParams.srv
  GetCPParams.srv
  SetCPCmd.srv
  SetARCParams.srv
  GetARCParams.srv
  SetARCCmd.srv
  SetWAITCmd.srv
  SetTRIGCmd.srv
  SetIOMultiplexing.srv
  GetIOMultiplexing.srv
  SetIODO.srv
  GetIODO.srv
  SetIOPWM.srv
  GetIOPWM.srv
  GetIODI.srv
  GetIOADC.srv
  SetEMotor.srv
  GetInfraredSensor.srv
  SetInfraredSensor.srv
  SetColorSensor.srv
  GetColorSensor.srv
  SetQueuedCmdStartExec.srv
  SetQueuedCmdStopExec.srv
  SetQueuedCmdForceStopExec.srv
  SetQueuedCmdClear.srv

  GraspService.srv
  ThrowService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs)

if (CMAKE_SIZEOF_VOID_P EQUAL 4)
    include_directories(
      ${catkin_INCLUDE_DIRS}
      src/DobotDll_x86
    )

    link_directories(
      src/DobotDll_x86
    )
else()
    include_directories(
      ${catkin_INCLUDE_DIRS}
      src/DobotDll_x64
    )

    link_directories(
      src/DobotDll_x64
    )
endif()

add_executable(DobotServer src/DobotServer.cpp)
target_link_libraries(DobotServer ${catkin_LIBRARIES} DobotDll)
add_dependencies(DobotServer dobot_gencpp)

add_executable(DobotClient_JOG src/DobotClient_JOG.cpp)
target_link_libraries(DobotClient_JOG ${catkin_LIBRARIES})
add_dependencies(DobotClient_JOG dobot_gencpp)

add_executable(DobotClient_PTP src/DobotClient_PTP.cpp)
target_link_libraries(DobotClient_PTP ${catkin_LIBRARIES})
add_dependencies(DobotClient_PTP dobot_gencpp)

install(PROGRAMS
  scripts/Action.py
  scripts/DobotUse.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)