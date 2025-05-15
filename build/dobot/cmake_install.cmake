# Install script for directory: /home/eaibot/nju_ws/src/dobot

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/eaibot/nju_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot/srv" TYPE FILE FILES
    "/home/eaibot/nju_ws/src/dobot/srv/SetCmdTimeout.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetDeviceSN.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetDeviceName.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetDeviceName.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetDeviceVersion.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetPose.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetAlarmsState.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/ClearAllAlarmsState.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetHOMEParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetHOMEParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetHOMECmd.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetEndEffectorParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetEndEffectorParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetEndEffectorLaser.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetEndEffectorLaser.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetEndEffectorSuctionCup.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetEndEffectorSuctionCup.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetEndEffectorGripper.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetEndEffectorGripper.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetJOGJointParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetJOGJointParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetJOGCoordinateParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetJOGCoordinateParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetJOGCommonParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetJOGCommonParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetJOGCmd.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetPTPJointParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetPTPJointParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetPTPCoordinateParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetPTPCoordinateParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetPTPJumpParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetPTPJumpParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetPTPCommonParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetPTPCommonParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetPTPCmd.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetCPParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetCPParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetCPCmd.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetARCParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetARCParams.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetARCCmd.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetWAITCmd.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetTRIGCmd.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetIOMultiplexing.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetIOMultiplexing.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetIODO.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetIODO.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetIOPWM.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetIOPWM.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetIODI.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetIOADC.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetEMotor.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetInfraredSensor.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetInfraredSensor.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetColorSensor.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GetColorSensor.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetQueuedCmdStartExec.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetQueuedCmdStopExec.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetQueuedCmdForceStopExec.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/SetQueuedCmdClear.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/GraspService.srv"
    "/home/eaibot/nju_ws/src/dobot/srv/ThrowService.srv"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot/cmake" TYPE FILE FILES "/home/eaibot/nju_ws/build/dobot/catkin_generated/installspace/dobot-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/eaibot/nju_ws/devel/include/dobot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/eaibot/nju_ws/devel/share/roseus/ros/dobot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/eaibot/nju_ws/devel/share/common-lisp/ros/dobot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/eaibot/nju_ws/devel/share/gennodejs/ros/dobot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/eaibot/nju_ws/devel/lib/python2.7/dist-packages/dobot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/eaibot/nju_ws/devel/lib/python2.7/dist-packages/dobot")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/eaibot/nju_ws/build/dobot/catkin_generated/installspace/dobot.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot/cmake" TYPE FILE FILES "/home/eaibot/nju_ws/build/dobot/catkin_generated/installspace/dobot-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot/cmake" TYPE FILE FILES
    "/home/eaibot/nju_ws/build/dobot/catkin_generated/installspace/dobotConfig.cmake"
    "/home/eaibot/nju_ws/build/dobot/catkin_generated/installspace/dobotConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dobot" TYPE FILE FILES "/home/eaibot/nju_ws/src/dobot/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/dobot" TYPE PROGRAM FILES
    "/home/eaibot/nju_ws/src/dobot/scripts/Action.py"
    "/home/eaibot/nju_ws/src/dobot/scripts/DobotUse.py"
    )
endif()

