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
import math

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
    
    def setPTPCoordinateParams(self, xv, rv, xa, ra):
        SetParamsReq = SetPTPCoordinateParamsRequest()
        SetParamsReq.xyzVelocity = xv
        SetParamsReq.xyzAcceleration = xa
        SetParamsReq.rVelocity = rv
        SetParamsReq.rAcceleration = ra
        self.setPTPCoordinateParamsClient.call(SetParamsReq)
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

    def setPose(self, x, y, z, r): #设置末端位置
        SetPoseReq = SetPTPCmdRequest()
        x, y = self.safepose(x, y)
        SetPoseReq.ptpMode = 1
        SetPoseReq.x = x
        SetPoseReq.y = y
        SetPoseReq.z = z
        SetPoseReq.r = r
        self.SetPTPCmdClient.call(SetPoseReq)
        tempPose = self.getPose()
        while abs(tempPose[0] - x) > 1 or abs(tempPose[1] - y) > 1 or abs(tempPose[2] - z) > 1 or abs(tempPose[3] - r) > 1:
            tempPose = self.getPose()
        return True
    
    def setHome(self): #复位
        SetHomeReq = SetHOMECmdRequest()
        self.SetHomeClient.call(SetHomeReq)
        return True
    
    def clearAlarmsState(self): #清除报警
        ClearReq = ClearAllAlarmsStateRequest()
        self.ClearAlarmsStateClient.call(ClearReq)
        return True
    
    def suckupObject(self): #吸取快递盒
        SuctionCupReq = SetEndEffectorSuctionCupRequest()
        SuctionCupReq.enableCtrl = 1
        SuctionCupReq.suck = 1
        SuctionCupReq.isQueued = True
        self.SuctionCupClient.call(SuctionCupReq)
        return True
    
    def releaseObject(self): #释放快递盒
        SuctionCupReq = SetEndEffectorSuctionCupRequest()
        SuctionCupReq.enableCtrl = 0
        SuctionCupReq.suck = 1
        SuctionCupReq.isQueued = True
        self.SuctionCupClient.call(SuctionCupReq)
        return True
    
    def safepose(self, x, y): #保证安全距离
        if abs(y) <= 220:
            x = max(x, -140)
        R = math.sqrt(x * x + y * y)
        if R >= 320:
            print('[WARN] Out of safe distance!')
            rospy.logwarn('Out of safe distance!')
            x = x * 320 / R
            y = y * 320 / R
        return x, y

    def setIOMultiplexing(self, address, multiplex): #设置IO复用
        clientReq = SetIOMultiplexingRequest()
        clientReq.address = address
        clientReq.multiplex = multiplex
        self.setIOMultiplexingClient.call(clientReq)
        return True
    
    def setIOPWM(self, pwm): #设置舵机PWM值
        clientReq = SetIOPWMRequest()
        clientReq.address = 8
        clientReq.frequency = self.frequency
        pwm = max(23.7, min(pwm, 100))
        clientReq.dutyCycle = pwm
        self.setIOPWMClient.call(clientReq)
        return True
    
    def setTheta(self, theta): #设置末端舵机角度(百分比)
        self.setIOPWM((90-23.7)*theta+23.7)
        return True
    
    def Throw(self, dir, height): #投掷快递盒
        self.setTheta(0)
        rospy.sleep(0.1)
        self.setPose(250,0,height+30,0)
        rospy.sleep(0.1)
        self.setPose(250,0,height,0)
        self.suckupObject()
        rospy.sleep(0.3)
        self.setPose(250,0,height+30,0)
        rospy.sleep(0.1)
        self.setTheta(1)
        self.setPose(0,-270*dir,50,0)
        rospy.sleep(0.1)
        self.releaseObject()
        rospy.sleep(0.4)
        self.setTheta(0)
        rospy.sleep(0.1)
        self.setPose(250,0,0,0)
        return True
    
    def ThrowBox(self, pos_z, mailbox_pos): #投掷快递盒
        if mailbox_pos == 1 and pos_z == 1:
            self.Throw(1, -30) #上方邮件投掷到右侧邮箱
            print('上方邮件投掷到右侧邮箱')
        elif mailbox_pos == 1 and pos_z == 0:
            self.Throw(1, -60) #下方邮件投掷到右侧邮箱
            print('下方邮件投掷到右侧邮箱')
        elif mailbox_pos == 0 and pos_z == 1:
            self.Throw(-1, -30) #上方邮件投掷到左侧邮箱
            print('上方邮件投掷到左侧邮箱')
        elif mailbox_pos == 0 and pos_z == 0:
            self.Throw(-1, -60) #下方邮件投掷到左侧邮箱
            print('下方邮件投掷到左侧邮箱')
        return True
    
    def Catch(self, height1, height2, error_x, error_y): #抓取快递盒(带误差参数)
        MAXX = 9
        MAXY = 10
        if error_x > MAXX:
            error_x = MAXX
        if error_x < -MAXX:
            error_x = -MAXX
        if error_y > MAXY:
            error_y = MAXY
        if error_y < -MAXY:
            error_y = -MAXY
        error_x = 10 * error_x
        error_y = 10 * error_y
        calculate_PWM1 = (math.degrees(math.atan2(300+error_y, 0+error_x))+90)/180
        calculate_PWM2 = (math.degrees(math.atan2(210+error_y, 0+error_x))+90)/180
        calculate_PWM = (calculate_PWM1 + calculate_PWM2) / 2
        self.setTheta(0)
        rospy.sleep(0.1)
        self.setPose(0+error_x,305,height1+18,0)
        rospy.sleep(0.1)
        self.setTheta(calculate_PWM)
        rospy.sleep(0.6)
        self.setPose(0+error_x,305,height1,0)
        self.suckupObject()
        rospy.sleep(0.3)
        self.setPose(0+error_x,305,height1+18,0)
        rospy.sleep(0.1)
        self.setPose(0+error_x,200,height1+18,0)
        rospy.sleep(0.1)
        self.setPose(0+error_x,143,height1/3.5-5,0)
        rospy.sleep(0.1)
        self.setPose(250,0,height1/3.5-5,0)
        self.setPose(250,0,height2,0)
        self.setTheta(0)
        rospy.sleep(0.6)
        self.releaseObject()
        rospy.sleep(0.1)
        return True
    
    def CatchBox(self, shelf_z, pos_z, error_x, error_y): #抓取快递盒
        if shelf_z == 1 and pos_z == 1:
            self.Catch(85, -10, error_x, error_y) #货架上层邮件抓取到上方
            print('货架上层邮件抓取到上方')
        elif shelf_z == 1 and pos_z == 0:
            self.Catch(85, -40, error_x, error_y) #货架上层邮件抓取到下方
            print('货架上层邮件抓取到下方')
        elif shelf_z == 0 and pos_z == 1:
            self.Catch(-30, -10, error_x, error_y) #货架下层邮件抓取到上方
            print('货架下层邮件抓取到上方')
        elif shelf_z == 0 and pos_z == 0:
            self.Catch(-30, -40, error_x, error_y) #货架下层邮件抓取到下方
            print('货架下层邮件抓取到下方')
        return True

if __name__ == '__main__':
    dobot = Dobot()
    # dobot.setHome()
    dobot.setPTPCommonParams(1000.0, 1000.0)
    dobot.setPTPCoordinateParams(1000, 1000, 200, 200)
    dobot.clearAlarmsState()
    dobot.setIOMultiplexing(8, 2)
    dobot.setPose(250,0,0,0)
    rospy.sleep(0.1)
    dobot.setTheta(0)

    xerror = 0
    yerror = 0
    while True:
        dobot.CatchBox(0, 0, xerror, yerror)
        rospy.sleep(0.5)
        dobot.CatchBox(0, 0, -xerror, yerror)
        rospy.sleep(0.5)
        xerror += 1

    # dobot.ThrowBox(1, 0) #把上层邮件投掷到左侧邮箱
    # dobot.ThrowBox(0, 0) #把下层邮件投掷到左侧邮箱