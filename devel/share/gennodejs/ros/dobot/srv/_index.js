
"use strict";

let SetEndEffectorGripper = require('./SetEndEffectorGripper.js')
let SetPTPJumpParams = require('./SetPTPJumpParams.js')
let SetJOGCmd = require('./SetJOGCmd.js')
let ThrowService = require('./ThrowService.js')
let SetCPCmd = require('./SetCPCmd.js')
let SetJOGJointParams = require('./SetJOGJointParams.js')
let SetHOMEParams = require('./SetHOMEParams.js')
let GetIODI = require('./GetIODI.js')
let SetEndEffectorSuctionCup = require('./SetEndEffectorSuctionCup.js')
let GetPTPCoordinateParams = require('./GetPTPCoordinateParams.js')
let SetQueuedCmdStopExec = require('./SetQueuedCmdStopExec.js')
let ClearAllAlarmsState = require('./ClearAllAlarmsState.js')
let GetDeviceName = require('./GetDeviceName.js')
let SetEndEffectorParams = require('./SetEndEffectorParams.js')
let SetCmdTimeout = require('./SetCmdTimeout.js')
let GetDeviceVersion = require('./GetDeviceVersion.js')
let SetHOMECmd = require('./SetHOMECmd.js')
let SetCPParams = require('./SetCPParams.js')
let SetARCParams = require('./SetARCParams.js')
let SetEMotor = require('./SetEMotor.js')
let GetPTPJumpParams = require('./GetPTPJumpParams.js')
let SetPTPJointParams = require('./SetPTPJointParams.js')
let SetQueuedCmdForceStopExec = require('./SetQueuedCmdForceStopExec.js')
let GetEndEffectorLaser = require('./GetEndEffectorLaser.js')
let SetTRIGCmd = require('./SetTRIGCmd.js')
let GetColorSensor = require('./GetColorSensor.js')
let SetJOGCommonParams = require('./SetJOGCommonParams.js')
let SetEndEffectorLaser = require('./SetEndEffectorLaser.js')
let GetEndEffectorParams = require('./GetEndEffectorParams.js')
let SetInfraredSensor = require('./SetInfraredSensor.js')
let SetPTPCmd = require('./SetPTPCmd.js')
let SetIODO = require('./SetIODO.js')
let SetPTPCommonParams = require('./SetPTPCommonParams.js')
let SetColorSensor = require('./SetColorSensor.js')
let GetInfraredSensor = require('./GetInfraredSensor.js')
let GetJOGJointParams = require('./GetJOGJointParams.js')
let SetIOMultiplexing = require('./SetIOMultiplexing.js')
let SetIOPWM = require('./SetIOPWM.js')
let GetJOGCommonParams = require('./GetJOGCommonParams.js')
let GetIOMultiplexing = require('./GetIOMultiplexing.js')
let SetDeviceName = require('./SetDeviceName.js')
let SetARCCmd = require('./SetARCCmd.js')
let SetQueuedCmdStartExec = require('./SetQueuedCmdStartExec.js')
let GetEndEffectorGripper = require('./GetEndEffectorGripper.js')
let GetPTPCommonParams = require('./GetPTPCommonParams.js')
let SetQueuedCmdClear = require('./SetQueuedCmdClear.js')
let GetDeviceSN = require('./GetDeviceSN.js')
let SetWAITCmd = require('./SetWAITCmd.js')
let GraspService = require('./GraspService.js')
let GetHOMEParams = require('./GetHOMEParams.js')
let SetJOGCoordinateParams = require('./SetJOGCoordinateParams.js')
let GetPTPJointParams = require('./GetPTPJointParams.js')
let GetCPParams = require('./GetCPParams.js')
let GetIOPWM = require('./GetIOPWM.js')
let GetIODO = require('./GetIODO.js')
let GetIOADC = require('./GetIOADC.js')
let GetPose = require('./GetPose.js')
let GetAlarmsState = require('./GetAlarmsState.js')
let SetPTPCoordinateParams = require('./SetPTPCoordinateParams.js')
let GetARCParams = require('./GetARCParams.js')
let GetCPCmd = require('./GetCPCmd.js')
let GetJOGCoordinateParams = require('./GetJOGCoordinateParams.js')
let GetEndEffectorSuctionCup = require('./GetEndEffectorSuctionCup.js')

module.exports = {
  SetEndEffectorGripper: SetEndEffectorGripper,
  SetPTPJumpParams: SetPTPJumpParams,
  SetJOGCmd: SetJOGCmd,
  ThrowService: ThrowService,
  SetCPCmd: SetCPCmd,
  SetJOGJointParams: SetJOGJointParams,
  SetHOMEParams: SetHOMEParams,
  GetIODI: GetIODI,
  SetEndEffectorSuctionCup: SetEndEffectorSuctionCup,
  GetPTPCoordinateParams: GetPTPCoordinateParams,
  SetQueuedCmdStopExec: SetQueuedCmdStopExec,
  ClearAllAlarmsState: ClearAllAlarmsState,
  GetDeviceName: GetDeviceName,
  SetEndEffectorParams: SetEndEffectorParams,
  SetCmdTimeout: SetCmdTimeout,
  GetDeviceVersion: GetDeviceVersion,
  SetHOMECmd: SetHOMECmd,
  SetCPParams: SetCPParams,
  SetARCParams: SetARCParams,
  SetEMotor: SetEMotor,
  GetPTPJumpParams: GetPTPJumpParams,
  SetPTPJointParams: SetPTPJointParams,
  SetQueuedCmdForceStopExec: SetQueuedCmdForceStopExec,
  GetEndEffectorLaser: GetEndEffectorLaser,
  SetTRIGCmd: SetTRIGCmd,
  GetColorSensor: GetColorSensor,
  SetJOGCommonParams: SetJOGCommonParams,
  SetEndEffectorLaser: SetEndEffectorLaser,
  GetEndEffectorParams: GetEndEffectorParams,
  SetInfraredSensor: SetInfraredSensor,
  SetPTPCmd: SetPTPCmd,
  SetIODO: SetIODO,
  SetPTPCommonParams: SetPTPCommonParams,
  SetColorSensor: SetColorSensor,
  GetInfraredSensor: GetInfraredSensor,
  GetJOGJointParams: GetJOGJointParams,
  SetIOMultiplexing: SetIOMultiplexing,
  SetIOPWM: SetIOPWM,
  GetJOGCommonParams: GetJOGCommonParams,
  GetIOMultiplexing: GetIOMultiplexing,
  SetDeviceName: SetDeviceName,
  SetARCCmd: SetARCCmd,
  SetQueuedCmdStartExec: SetQueuedCmdStartExec,
  GetEndEffectorGripper: GetEndEffectorGripper,
  GetPTPCommonParams: GetPTPCommonParams,
  SetQueuedCmdClear: SetQueuedCmdClear,
  GetDeviceSN: GetDeviceSN,
  SetWAITCmd: SetWAITCmd,
  GraspService: GraspService,
  GetHOMEParams: GetHOMEParams,
  SetJOGCoordinateParams: SetJOGCoordinateParams,
  GetPTPJointParams: GetPTPJointParams,
  GetCPParams: GetCPParams,
  GetIOPWM: GetIOPWM,
  GetIODO: GetIODO,
  GetIOADC: GetIOADC,
  GetPose: GetPose,
  GetAlarmsState: GetAlarmsState,
  SetPTPCoordinateParams: SetPTPCoordinateParams,
  GetARCParams: GetARCParams,
  GetCPCmd: GetCPCmd,
  GetJOGCoordinateParams: GetJOGCoordinateParams,
  GetEndEffectorSuctionCup: GetEndEffectorSuctionCup,
};
