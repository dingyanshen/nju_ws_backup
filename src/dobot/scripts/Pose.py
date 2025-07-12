#! /usr/bin/env python2.7
# -*- coding:utf-8 -*-

import rospy
from dobot.srv import GetPose, GetPoseRequest
from dobot.srv import SetPTPCmd, SetPTPCmdRequest
from dobot.srv import SetHOMECmd, SetHOMECmdRequest
from dobot.srv import SetEndEffectorSuctionCup, SetEndEffectorSuctionCupRequest
from dobot.srv import ClearAllAlarmsState, ClearAllAlarmsStateRequest
from dobot.srv import SetPTPCommonParams, SetPTPCommonParamsRequest
from dobot.srv import SetPTPCoordinateParams, SetPTPCoordinateParamsRequest
from dobot.srv import SetIOMultiplexing, SetIOMultiplexingRequest
from dobot.srv import SetIOPWM, SetIOPWMRequest
from dobot.srv import SetIODO
from dobot.srv import SetHOMEParams
from dobot.srv import GetIOPWM, GetIOPWMRequest
from dobot.srv import SetPTPCoordinateParams

class Dobot():
    def __init__(self):
        self.frequency = 500.0
        self.GetPoseClient = rospy.ServiceProxy('/DobotServer/GetPose', GetPose)
        self.GetIOPWMClient = rospy.ServiceProxy('DobotServer/GetIOPWM', GetIOPWM)
        self.SetPTPCmdClient = rospy.ServiceProxy('/DobotServer/SetPTPCmd', SetPTPCmd)
        self.SetHomeClient = rospy.ServiceProxy('/DobotServer/SetHOMECmd', SetHOMECmd)
        self.SuctionCupClient = rospy.ServiceProxy('DobotServer/SetEndEffectorSuctionCup', SetEndEffectorSuctionCup)
        self.ClearAlarmsStateClient = rospy.ServiceProxy('/DobotServer/ClearAllAlarmsState', ClearAllAlarmsState)
        self.setPTPParamsClient = rospy.ServiceProxy('/DobotServer/SetPTPCommonParams', SetPTPCommonParams)
        self.setIOMultiplexingClient = rospy.ServiceProxy('/DobotServer/SetIOMultiplexing', SetIOMultiplexing)
        self.setIOPWMClient = rospy.ServiceProxy('/DobotServer/SetIOPWM', SetIOPWM)
        self.setIODOClient = rospy.ServiceProxy('/DobotServer/SetIODO', SetIODO)
        self.setHOMEParamsClient = rospy.ServiceProxy('/DobotServer/SetHOMEParams', SetHOMEParams)
        self.setPTPCoordinateParamsClient = rospy.ServiceProxy('/DobotServer/SetPTPCoordinateParams', SetPTPCoordinateParams)
        try:
            self.GetPoseClient.wait_for_service(1.0)
        except(rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr('GetPose Service call failed: %s', e)
        try:
            self.GetIOPWMClient.wait_for_service(1.0)
        except(rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr('GetIOPWM Service call failed: %s', e)
        try:
            self.SetPTPCmdClient.wait_for_service(1.0)
        except(rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr('SetPTP Service call failed: %s', e)
        try:
            self.SetHomeClient.wait_for_service(1.0)
        except(rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr('SetHome Service call failed: %s', e)
        try:
            self.SuctionCupClient.wait_for_service(1.0)
        except(rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr('SuctionCup Service call failed: %s', e)
        try:
            self.ClearAlarmsStateClient.wait_for_service(1.0)
        except(rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr('ClearAlarmsState Service call failed: %s', e)
        try:
            self.setPTPParamsClient.wait_for_service(1.0)
        except(rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr('SetPTPParams Service call failed: %s', e)
        try:
            self.setIOMultiplexingClient.wait_for_service(1.0)
        except(rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr('SetIOMultiplexing Service call failed: %s', e)
        try:
            self.setIOPWMClient.wait_for_service(1.0)
        except(rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr('SetIOPWM Service call failed: %s', e)
        try:
            self.setIODOClient.wait_for_service(1.0)
        except(rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr('SetIODO Service call failed: %s', e)
        try:
            self.setHOMEParamsClient.wait_for_service(1.0)
        except(rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr('SetHOMEParams Service call failed: %s', e)
        try:
            self.setPTPCoordinateParamsClient.wait_for_service(1.0)
        except(rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr('setPTPCoordinateParams Service call failed: %s', e)

    def setPTPCommonParams(self, v, a):
        SetParamsReq = SetPTPCommonParamsRequest()
        SetParamsReq.velocityRatio = v
        SetParamsReq.accelerationRatio = a
        self.setPTPParamsClient.call(SetParamsReq)
        return True

    def getPose(self): #获取末端位置
        GetPoseReq = GetPoseRequest()
        pose = self.GetPoseClient.call(GetPoseReq)
        return pose.x, pose.y, pose.z, pose.r
    
    def getPWM(self): #获取舵机PWM值
        getPWMReq = GetIOPWMRequest()
        getPWMReq.address = 8
        result = self.GetIOPWMClient.call(getPWMReq)
        return result.dutyCycle

if __name__ == '__main__':
    dobot = Dobot()
    while True:
        pose = dobot.getPose()
        print('x: %.2f, y: %.2f, z: %.2f, r: %.2f' % (pose[0], pose[1], pose[2], pose[3]))
        pwm = dobot.getPWM()
        print('PWM: %.2f' % pwm)
        rospy.sleep(1)