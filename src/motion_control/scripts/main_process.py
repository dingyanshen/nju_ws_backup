#! /usr/bin/env python2.7
# -*- coding:utf-8 -*-

import rospy
import actionlib
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion
from actionlib_msgs.msg import *
from camera.srv import PhotoshelfService, PhotoboxService, PhotoService
from dobot.srv import GraspService, ThrowService
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from basic_move import BasicMove

class MainController:
    def __init__(self,position_path):
        rospy.init_node('main_controller')
        self.BM = BasicMove(detailInfo=True)
        self.position = self.loadToDict(position_path, mode="pose")
        self.move_base_AS = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        print("Connecting to move_base action server...")
        self.move_base_AS.wait_for_server(rospy.Duration(60))
        self.pub_initialpose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=20)
        print("Waiting for photo_shelf_service...")
        rospy.wait_for_service('photo_shelf_service')
        print("Waiting for photo_box_service...")
        rospy.wait_for_service('photo_box_service')
        print("Waiting for photo_service...")
        rospy.wait_for_service('photo_service')
        print("Waiting for dobot_grasp_service...")
        rospy.wait_for_service('dobot_grasp_service')
        print("Waiting for dobot_throw_service...")
        rospy.wait_for_service('dobot_throw_service')
        self.photo_shelf_proxy = rospy.ServiceProxy('photo_shelf_service', PhotoshelfService)
        self.photo_box_proxy = rospy.ServiceProxy('photo_box_service', PhotoboxService)
        self.photo_proxy = rospy.ServiceProxy('photo_service', PhotoService)
        self.grasp_proxy = rospy.ServiceProxy('dobot_grasp_service', GraspService)
        self.throw_proxy = rospy.ServiceProxy('dobot_throw_service', ThrowService)

        self.mail_table = [] # results.positions_z.positions_x
        self.mail_table_L = [] # 左侧货架
        self.mail_table_R = [] # 右侧货架
        self.mail_box = [] # box_id.result
        self.priority_provinces = [] # 省份优先级
        self.L_provinces = [] # 左侧省份
        self.R_provinces = [] # 右侧省份
        self.platform_state = 0 # 平台状态

        self.CURRENT_STATE = "INIT" # INIT 初始化 PHOTO 拍照 RUN 运行
        self.CURRENT_LOCATION = "INIT" # INIT 初始化 poseKey 坐标

    def debug_print(self): # 调试函数
        print("Debug Information—————————————————————————————————————————")
        print("Current State: " + str(self.CURRENT_STATE))
        print("Current Location: " + str(self.CURRENT_LOCATION))
        print("Mail Table: " + str(self.mail_table))
        print("Mail Table L: " + str(self.mail_table_L))
        print("Mail Table R: " + str(self.mail_table_R))
        print("Mail Box: " + str(self.mail_box))
        print("Priority Provinces: " + str(self.priority_provinces))
        print("Left Provinces: " + str(self.L_provinces))
        print("Right Provinces: " + str(self.R_provinces))
        print("Platform State: " + str(self.platform_state))
        print("——————————————————————————————————————————————————————————")

    def loadToDict(self, file_path, mode): # 导入相关参数
        print("Loading " + str(mode) + "...")
        dictionary = dict()
        file = open(file_path, "r")
        for line in file:
            data = line.strip().split()
            if mode == "pose" and len(data) == 5:
                key, px, py, qz, qw = data
                px, py, qz, qw = float(px), float(py), float(qz), float(qw)
                pose = Pose(Point(px, py, 0.0), Quaternion(0.0, 0.0, qz, qw))
                dictionary[key] = pose
        file.close()
        print("Load " + str(mode) + " successfully!")
        return dictionary
    
    def welcome(self): # 欢迎界面
        welcome_msg = [
            "  _   _         _      _   _   ",
            " | \ | |       | |    | | | |  ",
            " |  \| |    _  | |    | | | |  ",
            " | |\  |   | |_| |    | |_| |  ",
            " |_| \_|    \___/      \___/   ",
            "                               ",
            "南    雍    智    运    未    来\n",
        ]
        for line in welcome_msg:
            print(line)

    def end(self): # 结束界面
        end_msg = [
            "  _   _         _      _   _   ",
            " | \ | |       | |    | | | |  ",
            " |  \| |    _  | |    | | | |  ",
            " | |\  |   | |_| |    | |_| |  ",
            " |_| \_|    \___/      \___/   ",
            "                               ",
            "任    务    顺    利    完    成\n",
        ]
        for line in end_msg:
            print(line)

    def calibratePose(self, poseKey): # 校准位姿
        print("Calibrating pose " + str(poseKey) + "...")
        originalPose = PoseWithCovarianceStamped()
        originalPose.header.frame_id = "map"
        originalPose.header.stamp = rospy.Time.now()
        originalPose.pose.pose = self.position[poseKey]
        originalPose.pose.covariance[0] = 0.01
        originalPose.pose.covariance[6 * 1 + 1] = 0.01
        originalPose.pose.covariance[6 * 5 + 5] = 0.01
        for _ in range(20):
            self.pub_initialpose.publish(originalPose)
            rospy.sleep(0.1)
        rospy.sleep(0.5)
        print("Calibrate pose " + str(poseKey) + " successfully!")

    def navigate_posekey(self, poseKey): # 导航到指定位置外层
        if self.CURRENT_STATE == "INIT" or self.CURRENT_STATE == "PHOTO":
            self._navigate_posekey(poseKey)
        elif self.CURRENT_STATE == "RUN":
            if self.CURRENT_LOCATION == poseKey:
                pass

            elif self.CURRENT_LOCATION == "start": # 临时增加的特殊情况
                self._navigate_posekey(poseKey)

            elif self.CURRENT_LOCATION == "ARD1": # 右下拍照结束的特殊情况
                if poseKey == "CL1" or poseKey == "CL2" or poseKey == "CL3" or poseKey == "CL4" or poseKey == "CL5":
                    self._navigate_posekey("start_left")
                    self._navigate_posekey(poseKey)
                elif poseKey == "CL6" or poseKey == "CL7" or poseKey == "CL8" or poseKey == "CL9" or poseKey == "CL10":
                    self._navigate_posekey(poseKey)

            elif self.CURRENT_LOCATION == "CL1" or self.CURRENT_LOCATION == "CL2" or self.CURRENT_LOCATION == "CL3" or self.CURRENT_LOCATION == "CL4" or self.CURRENT_LOCATION == "CL5":
                # self.BM.moveRotate(135)
                # self.BM.moveForward(-0.2)
                if poseKey == "CL1" or poseKey == "CL2" or poseKey == "CL3" or poseKey == "CL4" or poseKey == "CL5":
                    self._navigate_posekey(poseKey)
                elif poseKey == "CL6" or poseKey == "CL7" or poseKey == "CL8" or poseKey == "CL9" or poseKey == "CL10":
                    self._navigate_posekey("start")
                    self._navigate_posekey(poseKey)
                elif poseKey == "LU1" or poseKey == "LU2" or poseKey == "LD1" or poseKey == "LD2" or poseKey == "U":
                    self._navigate_posekey(poseKey)
                elif poseKey == "RU1" or poseKey == "RU2":
                    self._navigate_posekey("start")
                    self._navigate_posekey(poseKey)
                elif poseKey == "RD1" or poseKey == "RD2":
                    self._navigate_posekey("start")
                    self._navigate_posekey("ARU2")
                    self._navigate_posekey(poseKey)

            elif self.CURRENT_LOCATION == "CL6" or self.CURRENT_LOCATION == "CL7" or self.CURRENT_LOCATION == "CL8" or self.CURRENT_LOCATION == "CL9" or self.CURRENT_LOCATION == "CL10":
                # self.BM.moveRotate(315)
                # self.BM.moveForward(-0.2)
                if poseKey == "CL1" or poseKey == "CL2" or poseKey == "CL3" or poseKey == "CL4" or poseKey == "CL5":
                    self._navigate_posekey("start_left")
                    self._navigate_posekey(poseKey)
                elif poseKey == "CL6" or poseKey == "CL7" or poseKey == "CL8" or poseKey == "CL9" or poseKey == "CL10":
                    self._navigate_posekey(poseKey)
                elif poseKey == "LU1" or poseKey == "LU2":
                    self._navigate_posekey("start_left")
                    self._navigate_posekey(poseKey)
                elif poseKey == "LD1" or poseKey == "LD2":
                    self._navigate_posekey("start_left")
                    self._navigate_posekey("temp_left")
                    self._navigate_posekey(poseKey)
                elif poseKey == "RU1" or poseKey == "RU2" or poseKey == "RD1" or poseKey == "RD2" or poseKey == "U":
                    self._navigate_posekey(poseKey)

            elif self.CURRENT_LOCATION == "LU1" or self.CURRENT_LOCATION == "LU2":
                self.BM.moveRotate(90)
                self.BM.moveForward(-0.2)
                if poseKey == "CL1" or poseKey == "CL2" or poseKey == "CL3" or poseKey == "CL4" or poseKey == "CL5":
                    self._navigate_posekey(poseKey)
                elif poseKey == "CL6" or poseKey == "CL7" or poseKey == "CL8" or poseKey == "CL9" or poseKey == "CL10":
                    self._navigate_posekey("start")
                    self._navigate_posekey(poseKey)
                elif poseKey == "LU1" or poseKey == "LU2" or poseKey == "LD1" or poseKey == "LD2" or poseKey == "U" or poseKey == "RU1" or poseKey == "RU2":
                    self._navigate_posekey(poseKey)
                elif poseKey == "RD1" or poseKey == "RD2":
                    self._navigate_posekey("start")
                    self._navigate_posekey("ARU2")
                    self._navigate_posekey(poseKey)

            elif self.CURRENT_LOCATION == "LD1" or self.CURRENT_LOCATION == "LD2":
                self.BM.moveRotate(20)
                self.BM.moveForward(-0.2)
                if poseKey == "CL1" or poseKey == "CL2" or poseKey == "CL3" or poseKey == "CL4" or poseKey == "CL5":
                    self._navigate_posekey(poseKey)
                elif poseKey == "CL6" or poseKey == "CL7" or poseKey == "CL8" or poseKey == "CL9" or poseKey == "CL10":
                    self._navigate_posekey("ALU2")
                    self._navigate_posekey("start")
                    self._navigate_posekey(poseKey)
                elif poseKey == "LU1" or poseKey == "LU2" or poseKey == "LD1" or poseKey == "LD2":
                    self._navigate_posekey(poseKey)
                elif poseKey == "RU1" or poseKey == "RU2" or poseKey == "U":
                    self._navigate_posekey("ALU2")
                    self._navigate_posekey("start")
                    self._navigate_posekey(poseKey)
                elif poseKey == "RD1" or poseKey == "RD2":
                    self._navigate_posekey("ALU2")
                    self._navigate_posekey("start")
                    self._navigate_posekey("ARU2")
                    self._navigate_posekey(poseKey)

            elif self.CURRENT_LOCATION == "RU1" or self.CURRENT_LOCATION == "RU2":
                if self.CURRENT_LOCATION == "RU1":
                    self.BM.moveForward(-0.3)
                self.BM.moveRotate(90)
                self.BM.moveForward(-0.2)
                if poseKey == "CL1" or poseKey == "CL2" or poseKey == "CL3" or poseKey == "CL4" or poseKey == "CL5":
                    self._navigate_posekey("start_left")
                    self._navigate_posekey(poseKey)
                elif poseKey == "CL6" or poseKey == "CL7" or poseKey == "CL8" or poseKey == "CL9" or poseKey == "CL10":
                    self._navigate_posekey(poseKey)
                elif poseKey == "LU1" or poseKey == "LU2" or poseKey == "RD1" or poseKey == "RD2" or poseKey == "U" or poseKey == "RU1" or poseKey == "RU2":
                    self._navigate_posekey(poseKey)
                elif poseKey == "LD1" or poseKey == "LD2":
                    self._navigate_posekey("start_left")
                    self._navigate_posekey("temp_left")
                    self._navigate_posekey(poseKey)

            elif self.CURRENT_LOCATION == "RD1" or self.CURRENT_LOCATION == "RD2":
                self.BM.moveRotate(160)
                self.BM.moveForward(-0.2)
                if poseKey == "CL1" or poseKey == "CL2" or poseKey == "CL3" or poseKey == "CL4" or poseKey == "CL5":
                    self._navigate_posekey("temp2_left")
                    self._navigate_posekey("start_left")
                    self._navigate_posekey(poseKey)
                elif poseKey == "CL6" or poseKey == "CL7" or poseKey == "CL8" or poseKey == "CL9" or poseKey == "CL10":
                    self._navigate_posekey(poseKey)
                elif poseKey == "RU1" or poseKey == "RU2" or poseKey == "RD1" or poseKey == "RD2":
                    self._navigate_posekey(poseKey)
                elif poseKey == "LU1" or poseKey == "LU2" or poseKey == "U":
                    self._navigate_posekey("temp2_left")
                    self._navigate_posekey("start_left")
                    self._navigate_posekey(poseKey)
                elif poseKey == "LD1" or poseKey == "LD2":
                    self._navigate_posekey("temp2_left")
                    self._navigate_posekey("start_left")
                    self._navigate_posekey("temp_left")
                    self._navigate_posekey(poseKey)

            elif self.CURRENT_LOCATION == "U":
                self.BM.moveRotate(90)
                self.BM.moveForward(-0.2)
                if poseKey == "CL1" or poseKey == "CL2" or poseKey == "CL3" or poseKey == "CL4" or poseKey == "CL5":
                    self._navigate_posekey(poseKey)
                elif poseKey == "CL6" or poseKey == "CL7" or poseKey == "CL8" or poseKey == "CL9" or poseKey == "CL10":
                    self._navigate_posekey(poseKey)
                elif poseKey == "RU1" or poseKey == "RU2" or poseKey == "LU1" or poseKey == "LU2" or poseKey == "U":
                    self._navigate_posekey(poseKey)
                elif poseKey == "RD1" or poseKey == "RD2":
                    self._navigate_posekey("ARU2")
                    self._navigate_posekey(poseKey)
                elif poseKey == "LD1" or poseKey == "LD2":
                    self._navigate_posekey("temp_left")
                    self._navigate_posekey(poseKey)
            else:
                self._navigate_posekey(poseKey)
            
        self.CURRENT_LOCATION = poseKey

    def _navigate_posekey(self, poseKey): # 导航到指定位置函数原型
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.position[poseKey]
        self.move_base_AS.send_goal(goal)
        while not self.move_base_AS.wait_for_result():
            pass

    def takeboxPic_RU(self): # 邮箱文字识别[右上]
        self.navigate_posekey("ARU2")
        print("ARU2 is arrived.")
        self.process_box("RU2")

        self.navigate_posekey("ARU1")
        print("ARU1 is arrived.")
        self.process_box("RU1")
        self.BM.moveForward(-0.3)

    def takeboxPic_RD(self): # 邮箱文字识别[右下]
        self.navigate_posekey("ARD2")
        print("ARD2 is arrived.")
        self.process_box("RD2")

        self.navigate_posekey("ARD1")
        print("ARD1 is arrived.")
        self.process_box("RD1")
        
    def takeboxPic_LU(self): # 邮箱文字识别[左上]
        self.navigate_posekey("ALU1")
        print("ALU1 is arrived.")
        self.process_box("LU1")

        self.navigate_posekey("ALU2")
        print("ALU2 is arrived.")
        self.process_box("LU2")

    def takeboxPic_LD(self): # 邮箱文字识别[左下]
        self.navigate_posekey("ALD1")
        print("ALD1 is arrived.")
        self.process_box("LD1")

        self.navigate_posekey("ALD2")
        print("ALD2 is arrived.")
        self.process_box("LD2")
                
    def takeshelfPic_R(self): # 货架拍照[右侧]
        self.navigate_posekey("RP3")
        print("RP3 is arrived.")
        self.process_shelf(4)
        
        self.navigate_posekey("RP2")
        print("RP2 is arrived.")
        self.process_shelf(5)
        
        self.navigate_posekey("RP1")
        print("RP1 is arrived.")
        self.process_shelf(6)

    def takeshelfPic_L(self): # 货架拍照[左侧]
        self.navigate_posekey("LP3")
        print("LP3 is arrived.")
        self.process_shelf(1)

        self.navigate_posekey("LP2")
        print("LP2 is arrived.")
        self.process_shelf(2)
        
        self.navigate_posekey("LP1")
        print("LP1 is arrived.")
        self.process_shelf(3)

    def process_shelf(self, type): # 货架拍照服务
        try:
            response = self.photo_shelf_proxy(type)
            print(response)
            if type == 1 or type == 2:
                for i in range(4):
                    self.mail_table.append({
                        'results': response.results[i],
                        'positions_z': response.positions_z[i],
                        'positions_x': response.positions_x[i],
                    })
                for i in range(4):
                    self.mail_table_L.append({
                        'results': response.results[i],
                        'positions_z': response.positions_z[i],
                        'positions_x': response.positions_x[i],
                    })
            elif type == 5 or type == 6:
                for i in range(4):
                    self.mail_table.append({
                        'results': response.results[i],
                        'positions_z': response.positions_z[i],
                        'positions_x': response.positions_x[i],
                    })
                for i in range(4):
                    self.mail_table_R.append({
                        'results': response.results[i],
                        'positions_z': response.positions_z[i],
                        'positions_x': response.positions_x[i],
                    })
            elif type == 3:
                self.mail_table.append({
                    'results': response.results[0],
                    'positions_z': response.positions_z[0],
                    'positions_x': response.positions_x[0],
                })
                self.mail_table.append({
                    'results': response.results[2],
                    'positions_z': response.positions_z[2],
                    'positions_x': response.positions_x[2],
                })
                self.mail_table_L.append({
                    'results': response.results[0],
                    'positions_z': response.positions_z[0],
                    'positions_x': response.positions_x[0],
                })
                self.mail_table_L.append({
                    'results': response.results[2],
                    'positions_z': response.positions_z[2],
                    'positions_x': response.positions_x[2],
                })
            elif type == 4:
                self.mail_table.append({
                    'results': response.results[1],
                    'positions_z': response.positions_z[1],
                    'positions_x': response.positions_x[1],
                })
                self.mail_table.append({
                    'results': response.results[3],
                    'positions_z': response.positions_z[3],
                    'positions_x': response.positions_x[3],
                })
                self.mail_table_R.append({
                    'results': response.results[1],
                    'positions_z': response.positions_z[1],
                    'positions_x': response.positions_x[1],
                })
                self.mail_table_R.append({
                    'results': response.results[3],
                    'positions_z': response.positions_z[3],
                    'positions_x': response.positions_x[3],
                })
        except rospy.ServiceException as e:
            rospy.logerr("Photo service call failed: {e}")

    def process_box(self, box_id): # 邮箱拍照服务
        try:
            response = self.photo_box_proxy()
            print(response.result)
            self.mail_box.append({
                    'box_id': box_id,
                    'result': response.result
                })
            self.priority_provinces.append(response.result)
            if box_id == "LU1" or box_id == "LU2" or box_id == "LD1" or box_id == "LD2":
                self.L_provinces.append(response.result)
            elif box_id == "RU1" or box_id == "RU2" or box_id == "RD1" or box_id == "RD2":
                self.R_provinces.append(response.result)
        except rospy.ServiceException as e:
            rospy.logerr("Photo service call failed: {e}")
    
    def grasp_mail(self, mail): # 抓取邮件
        try:
            self.navigate_posekey("CL" + str(mail['positions_x']))
            if mail['positions_z'] == 1 and self.platform_state == 0: # 上层抓到下层
                response = self.photo_proxy(mail['positions_x'])
                catch_type = [1, 0, response.error_x, response.error_y]
                response = self.grasp_proxy(*catch_type)
                self.platform_state = 1
            elif mail['positions_z'] == 1 and self.platform_state == 1: # 上层抓到上层
                response = self.photo_proxy(mail['positions_x'])
                catch_type = [1, 1, response.error_x, response.error_y]
                response = self.grasp_proxy(*catch_type)
                self.platform_state = 2
            elif mail['positions_z'] == 2 and self.platform_state == 0: # 下层抓到下层
                response = self.photo_proxy(mail['positions_x']+10)
                catch_type = [0, 0, response.error_x, response.error_y]
                response = self.grasp_proxy(*catch_type)
                self.platform_state = 1
            elif mail['positions_z'] == 2 and self.platform_state == 1: # 下层抓到上层
                response = self.photo_proxy(mail['positions_x']+10)
                catch_type = [0, 1, response.error_x, response.error_y]
                response = self.grasp_proxy(*catch_type)
                self.platform_state = 2
            else:
                rospy.logwarn("邮件位置不正确")
                return False
            if response.success:
                return True
            else:
                rospy.logwarn("抓取邮件失败")
                return False
        except rospy.ServiceException as e:
            rospy.logerr("抓取服务调用失败: {e}")
            return False
        
    def deliver_mails(self, mails): # 运送邮件
        mails.reverse()
        try:
            for mail in mails:
                found = False
                for box in self.mail_box:
                    print(box)
                    print(mail)
                    if box['result'] == mail['results']:
                        print("运送邮件" + str(mail['results']) + "到邮箱" + str(box['box_id']))
                        self.navigate_posekey(box['box_id'])
                        found = True
                        break
                if not found:
                    print("未找到对应的邮箱，无法运送邮件" + str(mail['results']))
                    print("运送邮件" + str(mail['results']) + "到无效邮箱U")
                    self.navigate_posekey("U")
                if self.platform_state == 2: # 上层
                    throw_type = [1, 0]
                    response = self.throw_proxy(*throw_type)
                    self.platform_state = 1
                elif self.platform_state == 1: # 下层
                    throw_type = [0, 0]
                    response = self.throw_proxy(*throw_type)
                    self.platform_state = 0
                if not response.success:
                    rospy.logwarn("运送邮件失败")
                    return False
            return True
        except rospy.ServiceException as e:
            rospy.logerr("运送服务调用失败: {e}")
            return False
        
    def select_nearest_province(self, mail_table, priority_provinces):  # 对mail_table按照priority_provinces重新排序
        flattened_mails = mail_table
        priority_mails = [mail for mail in flattened_mails if mail['results'] in priority_provinces]
        other_mails = [mail for mail in flattened_mails if mail['results'] not in priority_provinces]
        sorted_mail_table = priority_mails + other_mails
        return sorted_mail_table
    
    def process_priority_mails(self, mail_table, priority_provinces): # 处理优先省份邮件
        grabbed_mails = []

        for mail in mail_table:
            if mail['results'] not in priority_provinces:
                break
            if self.grasp_mail(mail):
                grabbed_mails.append(mail)

                if len(grabbed_mails) >= 2:
                    self.deliver_mails(grabbed_mails)
                    mail_table = [mail for mail in mail_table if mail not in grabbed_mails]
                    grabbed_mails = []

        if grabbed_mails:
            self.deliver_mails(grabbed_mails)
            mail_table = [mail for mail in mail_table if mail not in grabbed_mails]
            grabbed_mails = []
        return mail_table

    def process_non_priority_mails(self, mail_table): # 处理非优先省份邮件
        grabbed_mails = []
        for mail in mail_table:
            if self.grasp_mail(mail):
                grabbed_mails.append(mail)

                if len(grabbed_mails) >= 2:
                    self.deliver_mails(grabbed_mails)
                    mail_table = [mail for mail in mail_table if mail not in grabbed_mails]
                    grabbed_mails = []

        if grabbed_mails:
            self.deliver_mails(grabbed_mails)
            mail_table = [mail for mail in mail_table if mail not in grabbed_mails]
            grabbed_mails = []

    def AutoSet(self): # 测试用
        self.mail_box.append({'box_id': "LU1", 'result': 1})
        self.mail_box.append({'box_id': "LU2", 'result': 2})
        self.mail_box.append({'box_id': "LD1", 'result': 3})
        self.mail_box.append({'box_id': "LD2", 'result': 4})
        self.mail_box.append({'box_id': "RU1", 'result': 5})
        self.mail_box.append({'box_id': "RU2", 'result': 6})
        self.mail_box.append({'box_id': "RD1", 'result': 7})
        self.mail_box.append({'box_id': "RD2", 'result': 8})
        
        self.priority_provinces.append(1)
        self.priority_provinces.append(2)
        self.priority_provinces.append(3)
        self.priority_provinces.append(4)
        self.priority_provinces.append(5)
        self.priority_provinces.append(6)
        self.priority_provinces.append(7)
        self.priority_provinces.append(8)

        self.L_provinces.append(1)
        self.L_provinces.append(2)
        self.L_provinces.append(3)
        self.L_provinces.append(4)
        self.R_provinces.append(5)
        self.R_provinces.append(6)
        self.R_provinces.append(7)
        self.R_provinces.append(8)
        
        self.mail_table.append({'results': 0,'positions_z': 1,'positions_x': 1,})
        self.mail_table.append({'results': 0,'positions_z': 1,'positions_x': 2,})
        self.mail_table.append({'results': 0,'positions_z': 1,'positions_x': 3,})
        self.mail_table.append({'results': 0,'positions_z': 1,'positions_x': 4,})
        self.mail_table.append({'results': 1,'positions_z': 1,'positions_x': 5,})
        self.mail_table.append({'results': 1,'positions_z': 1,'positions_x': 6,})
        self.mail_table.append({'results': 2,'positions_z': 1,'positions_x': 7,})
        self.mail_table.append({'results': 2,'positions_z': 1,'positions_x': 8,})
        self.mail_table.append({'results': 3,'positions_z': 1,'positions_x': 9,})
        self.mail_table.append({'results': 3,'positions_z': 1,'positions_x': 10,})

        self.mail_table.append({'results': 4,'positions_z': 2,'positions_x': 1,})
        self.mail_table.append({'results': 4,'positions_z': 2,'positions_x': 2,})
        self.mail_table.append({'results': 5,'positions_z': 2,'positions_x': 3,})
        self.mail_table.append({'results': 5,'positions_z': 2,'positions_x': 4,})
        self.mail_table.append({'results': 6,'positions_z': 2,'positions_x': 5,})
        self.mail_table.append({'results': 6,'positions_z': 2,'positions_x': 6,})
        self.mail_table.append({'results': 7,'positions_z': 2,'positions_x': 7,})
        self.mail_table.append({'results': 7,'positions_z': 2,'positions_x': 8,})
        self.mail_table.append({'results': 8,'positions_z': 2,'positions_x': 9,})
        self.mail_table.append({'results': 8,'positions_z': 2,'positions_x': 10,})

        self.mail_table_L.append({'results': 0,'positions_z': 1,'positions_x': 1,})
        self.mail_table_L.append({'results': 0,'positions_z': 1,'positions_x': 2,})
        self.mail_table_L.append({'results': 0,'positions_z': 1,'positions_x': 3,})
        self.mail_table_L.append({'results': 0,'positions_z': 1,'positions_x': 4,})
        self.mail_table_L.append({'results': 1,'positions_z': 1,'positions_x': 5,})
        self.mail_table_R.append({'results': 1,'positions_z': 1,'positions_x': 6,})
        self.mail_table_R.append({'results': 2,'positions_z': 1,'positions_x': 7,})
        self.mail_table_R.append({'results': 2,'positions_z': 1,'positions_x': 8,})
        self.mail_table_R.append({'results': 3,'positions_z': 1,'positions_x': 9,})
        self.mail_table_R.append({'results': 3,'positions_z': 1,'positions_x': 10,})

        self.mail_table_L.append({'results': 4,'positions_z': 2,'positions_x': 1,})
        self.mail_table_L.append({'results': 4,'positions_z': 2,'positions_x': 2,})
        self.mail_table_L.append({'results': 5,'positions_z': 2,'positions_x': 3,})
        self.mail_table_L.append({'results': 5,'positions_z': 2,'positions_x': 4,})
        self.mail_table_L.append({'results': 6,'positions_z': 2,'positions_x': 5,})
        self.mail_table_R.append({'results': 6,'positions_z': 2,'positions_x': 6,})
        self.mail_table_R.append({'results': 7,'positions_z': 2,'positions_x': 7,})
        self.mail_table_R.append({'results': 7,'positions_z': 2,'positions_x': 8,})
        self.mail_table_R.append({'results': 8,'positions_z': 2,'positions_x': 9,})
        self.mail_table_R.append({'results': 8,'positions_z': 2,'positions_x': 10,})

    def run(self):
        self.welcome() # 欢迎界面

        self.calibratePose("start_left") # 校准起始位姿

        self.CURRENT_LOCATION = "start_left"

        self.mail_box.append({'box_id': "U",'result': 0}) # 无效箱子
        self.priority_provinces.append(0) # 无效箱子优先
        self.L_provinces.append(0) # 无效箱子优先
        self.R_provinces.append(0) # 无效箱子优先

        self.CURRENT_STATE = "PHOTO"

        self.takeboxPic_LD() # 邮箱拍照[左下]
        self.takeshelfPic_L() # 货架拍照[左侧]
        self.takeboxPic_LU() # 邮箱拍照[左上]
        self.takeboxPic_RU() # 邮箱拍照[右上]
        self.takeshelfPic_R() # 货架拍照[右侧]
        self.takeboxPic_RD() # 邮箱拍照[右下]

        self.navigate_posekey("start")

        # self.AutoSet()

        self.CURRENT_STATE = "RUN"

        self.mail_table_R = self.select_nearest_province(self.mail_table_R, self.R_provinces) # 对mail_table按照priority_provinces重新排序
        print("开始处理右侧优先省份邮件...")
        self.mail_table_R = self.process_priority_mails(self.mail_table_R, self.R_provinces) # 处理右侧优先省份邮件

        self.mail_table_L = self.select_nearest_province(self.mail_table_L, self.L_provinces) # 对mail_table按照priority_provinces重新排序
        print("开始处理左侧优先省份邮件...")
        self.mail_table_L = self.process_priority_mails(self.mail_table_L, self.L_provinces) # 处理左侧优先省份邮件

        self.mail_table = [] # 清空邮件列表
        self.mail_table = self.mail_table_L + self.mail_table_R # 合并邮件列表

        print("开始处理非优先省份邮件...")
        self.process_non_priority_mails(self.mail_table) # 处理非优先省份邮件

        self.end() # 结束界面
        
if __name__ == "__main__":
    position_path = "/home/eaibot/nju_ws/src/motion_control/config/position.txt"
    controller = MainController(position_path)
    controller.run()