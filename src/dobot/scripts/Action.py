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
        x, y= self.safepose(x, y, z)
        SetPoseReq = SetPTPCmdRequest()
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

    def safepose(self, x, y, z): #保证安全距离 内层约束
        # 上圆弧约束
        R = math.sqrt(x * x + y * y) # 机械臂半径
        if R >= 300 and z >= 90: # 高处半径过大
            print("过大半径：" + str(x) + " " + str(y))
            x = x * 300 / R
            y = y * 300 / R
            print("调整半径300：" + str(x) + " " + str(y))
        elif R >= 320 and z < 90: # 低处半径过大
            print("过大半径：" + str(x) + " " + str(y))
            x = x * 320 / R
            y = y * 320 / R
            print("调整半径320：" + str(x) + " " + str(y))
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
        pwm = self.between(20.0, pwm, 100.0) # 20.0% ~ 100.0% 为安全限制范围
        clientReq.dutyCycle = pwm
        self.setIOPWMClient.call(clientReq)
        return True
    
    def setTheta(self, theta): #设置末端舵机角度百分比
        self.setIOPWM(25.1 + (91.0 - 25.1) * theta) # 25.1% ~ 91.0% 为实际范围
        return True

    def CatchBox(self, shelf_z, pos_z, error_x, error_y): #抓取快递盒外层
        # 根据货架位置shelf_z(1-上层 0-下层) 平台位置pos_z(1-上方 0-下方) error_x error_y
        UPPER_PLATFORM_HEIGHT = -10
        LOWER_PLATFORM_HEIGHT = -40
        if shelf_z == 1 and pos_z == 1:
            self.CatchUP(UPPER_PLATFORM_HEIGHT, error_x, error_y) #货架上层邮件抓取到上方
            print('货架上层邮件抓取到上方')
        elif shelf_z == 1 and pos_z == 0:
            self.CatchUP(LOWER_PLATFORM_HEIGHT, error_x, error_y) #货架上层邮件抓取到下方
            print('货架上层邮件抓取到下方')
        elif shelf_z == 0 and pos_z == 1:
            self.CatchDOWN(UPPER_PLATFORM_HEIGHT, error_x, error_y) #货架下层邮件抓取到上方
            print('货架下层邮件抓取到上方')
        elif shelf_z == 0 and pos_z == 0:
            self.CatchDOWN(LOWER_PLATFORM_HEIGHT, error_x, error_y) #货架下层邮件抓取到下方
            print('货架下层邮件抓取到下方')
        return True

    def ThrowBox(self, pos_z, mailbox_pos): #投掷快递盒外层
        # 根据平台位置pos_z(1-上方 0-下方) 邮箱位置mailbox_pos(1-右侧 0-左侧)
        UPPER_HEIGHT = -30
        LOWER_HEIGHT = -60
        if mailbox_pos == 1 and pos_z == 1:
            self.Throw(1, UPPER_HEIGHT) #上方邮件投掷到右侧邮箱
            print('上方邮件投掷到右侧邮箱')
        elif mailbox_pos == 1 and pos_z == 0:
            self.Throw(1, LOWER_HEIGHT) #下方邮件投掷到右侧邮箱
            print('下方邮件投掷到右侧邮箱')
        elif mailbox_pos == 0 and pos_z == 1:
            self.Throw(-1, UPPER_HEIGHT) #上方邮件投掷到左侧邮箱
            print('上方邮件投掷到左侧邮箱')
        elif mailbox_pos == 0 and pos_z == 0:
            self.Throw(-1, LOWER_HEIGHT) #下方邮件投掷到左侧邮箱
            print('下方邮件投掷到左侧邮箱')
        return True

    def between(self, min_x, value_x, max_x): #限制值范围
        return min(max(value_x, min_x), max_x)

    def _calculate_error(self, error_x, error_y, mode): #计算右移量和伸展量mm
        # 此处传入的误差参数是已经判断可以抓取的安全误差参数
        # 此处为第二次限定 目的是将临边界外的点限制在安全范围内
        if mode == 'UP': # 上层
            error_x = self.between(-10.0, error_x, 10.0) # 右移量限制在-10cm到10cm之间
            error_y = self.between(-6.5, error_y, 4.1) # 伸展量限制在-6.5cm到4.1cm之间
        elif mode == 'DOWN': # 下层
            error_x = self.between(-10.0, error_x, 10.0) # 右移量限制在-10cm到10cm之间
            error_y = self.between(-5.5, error_y, 6.0) # 伸展量限制在-5.5cm到6.0cm之间
        error_x *= 10 # 右移量转换为mm
        error_y *= 10 # 伸展量转换为mm
        print("调整参数：" + str(error_x) + " " + str(error_y))
        return error_x, error_y
    
    def _calculate_rotation(self, error_x, error_y): #计算舵机旋转百分比 输入mm
        max_stretch = math.atan2(300+error_y, 0+error_x) # 最大伸展大致 300mm
        min_stretch = math.atan2(200+error_y, 0+error_x) # 最小伸展大致 200mm
        max_rotation = (math.degrees(max_stretch) + 90) / 180 # 最大伸展
        min_rotation = (math.degrees(min_stretch) + 90) / 180 # 最小伸展
        rotation = (max_rotation + min_rotation) / 2 # 平均伸展
        return rotation

    def CatchUP(self, height, error_x, error_y):
        HEIGHT_UP = 100 # 货架上层高度mm
        error_x, error_y = self._calculate_error(error_x, error_y, mode='UP') # 计算右移量和伸展量mm
        rotation = self._calculate_rotation(error_x, error_y) # 计算舵机旋转百分比

        mail_x = error_x # 邮件右移 0mm
        mail_y = 260 + error_y # 邮件伸展 260mm

        # 抬到默认位
        self.setPose(250, 0, -10, 0)

        # 舵机内缩
        self.setTheta(0)
        rospy.sleep(0.1)

        # 达到 (mail_x, mail_y, +20)
        self.setPose(mail_x, mail_y, HEIGHT_UP+20, 0)
        rospy.sleep(0.1)

        # 舵机外伸到计算角
        self.setTheta(rotation)
        rospy.sleep(0.8)

        # 下压 (mail_x, mail_y, +0) 吸取
        self.setPose(mail_x, mail_y, HEIGHT_UP, 0)
        self.suckupObject()
        rospy.sleep(0.3)

        # 抬起 (mail_x, mail_y, +20)
        self.setPose(mail_x, mail_y, HEIGHT_UP+20, 0)
        rospy.sleep(0.1)

        # 抖动旋转避让 (mail_x, 195, +20) (mail_x+12, 195, +20) (mail_x+24, 195, +20)
        self.setPose(mail_x, 195, HEIGHT_UP+20, 0)
        self.setTheta(rotation)
        self.setPose(12+mail_x, 195, HEIGHT_UP+20, 0)
        self.setTheta(rotation - 0.08)
        self.setPose(24+mail_x, 195, HEIGHT_UP+20, 0)
        self.setTheta(0.3)
        rospy.sleep(0.1)
        
        # 转到平台上方 (250, 0, +20)
        self.setPose(250, 0, HEIGHT_UP+20, 0)

        # 舵机内缩
        self.setTheta(0)

        # 达到平台高度
        self.setPose(250, 0, height, 0)
        rospy.sleep(0.8)

        # 释放
        self.releaseObject()
        rospy.sleep(0.1)

        # 抓取完防滑下压
        self.setPose(250, 0, height-17, 0)

        return True
  
    def CatchDOWN(self, height, error_x, error_y):
        HEIGHT_DOWN = -15 # 货架下层高度mm
        error_x, error_y = self._calculate_error(error_x, error_y, mode='DOWN') # 计算右移量和伸展量mm
        rotation = self._calculate_rotation(error_x, error_y) # 计算舵机旋转百分比

        mail_x = error_x # 邮件右移 0mm
        mail_y = 260 + error_y # 邮件伸展 260mm

        # 抬到默认位
        self.setPose(250, 0, -10, 0)
        
        # 舵机内缩
        self.setTheta(0)
        rospy.sleep(0.1)

        # 达到 (mail_x, mail_y, +20)
        self.setPose(mail_x, mail_y, HEIGHT_DOWN+20, 0)
        rospy.sleep(0.1)

        # 舵机外伸到计算角
        self.setTheta(rotation)
        rospy.sleep(0.8)

        # 下压 (mail_x, mail_y, +0) 吸取
        self.setPose(mail_x, mail_y, HEIGHT_DOWN, 0)
        self.suckupObject()
        rospy.sleep(0.3)

        # 抬起 (mail_x, mail_y, +20)
        self.setPose(mail_x, mail_y, HEIGHT_DOWN+20, 0)
        rospy.sleep(0.1)

        # 抖动旋转避让 (mail_x, 195, +20) (mail_x+12, 195, +20) (mail_x+24, 195, +20)
        self.setPose(mail_x, 195, HEIGHT_DOWN+20, 0)
        self.setTheta(rotation)
        self.setPose(12+mail_x, 195, HEIGHT_DOWN+20, 0)
        self.setTheta(rotation - 0.08)
        self.setPose(24+mail_x, 195, HEIGHT_DOWN+20, 0)
        self.setTheta(0.4)
        rospy.sleep(0.1)

        # 转到平台上方略伸展 (280, 0, +20)
        self.setPose(280, 0, HEIGHT_DOWN+20, 0)

        # 舵机内缩
        self.setTheta(0)

        # 达到平台高度
        self.setPose(250, 0, height, 0)
        rospy.sleep(0.8)

        # 释放
        self.releaseObject()
        rospy.sleep(0.1)

        # 抓取完防滑下压
        self.setPose(250, 0, height-17, 0)

        return True

    def Throw(self, dir, height): #投掷快递盒

        # 抬到默认位
        self.setPose(250, 0, -10, 0)

        # 舵机内缩
        self.setTheta(0)
        rospy.sleep(0.1)

        # 下压 吸取
        self.setPose(250, 0, height, 0)
        self.suckupObject()
        rospy.sleep(0.3)

        # 抬起
        self.setPose(250, 0, height+20, 0)
        rospy.sleep(0.1)

        # 舵机外伸 转向邮箱
        self.setTheta(1)
        self.setPose(0, -320*dir, 50, 0)
        rospy.sleep(0.1)

        # 释放
        self.releaseObject()
        rospy.sleep(0.4)

        # 舵机内缩 回到默认位
        self.setTheta(0)
        rospy.sleep(0.1)
        self.setPose(250, 0, -10, 0)

        # 投掷完防滑下压 只有投掷上层邮件后需要 投掷下层邮件不需要
        self.setPose(250, 0, -57, 0)
        
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