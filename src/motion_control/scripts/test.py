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
    def __init__(self, position_path):
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

    def navigate_posekey(self, poseKey): # 导航到指定位置
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.position[poseKey]
        self.move_base_AS.send_goal(goal)
        while not self.move_base_AS.wait_for_result():
            pass

    def test_photo_shelf(self, type):
        response = self.photo_shelf_proxy(type)
        print(response)

    def test_photo_box(self):
        response = self.photo_box_proxy()
        print(response)

    def test_grasp(self, type):
        if type == 1:
            response = self.photo_proxy(2)
            catch_type = [0, 0, response.error_x, response.error_y]
        elif type == 2:
            response = self.photo_proxy(2)
            catch_type = [0, 1, response.error_x, response.error_y]
        elif type == 3:
            response = self.photo_proxy(1)
            catch_type = [1, 0, response.error_x, response.error_y]
        elif type == 4:
            response = self.photo_proxy(1)
            catch_type = [1, 1, response.error_x, response.error_y]
        response = self.grasp_proxy(*catch_type)
        print(response)

    def test_throw(self, type):
        if type == 1:
            throw_type = [0, 0]
        elif type == 2:
            throw_type = [0, 1]
        elif type == 3:
            throw_type = [1, 0]
        elif type == 4:
            throw_type = [1, 1]
        response = self.throw_proxy(*throw_type)
        print(response)
        
if __name__ == "__main__":
    position_path = "/home/eaibot/nju_ws/src/motion_control/config/position.txt"
    controller = MainController(position_path)
    controller.calibratePose("start_left")

    while True:
        choice = raw_input("请输入选择 s拍货架 b拍邮箱 c抓取 t投递 n导航 z自转 p校准位姿 l加载位置 q退出：")
        if choice == "s":
            type = input("请输入类型（1-6）：")
            if type != 1 and type != 2 and type != 3 and type != 4 and type != 5 and type != 6:
                print("输入错误，请重新输入")
                continue
            controller.test_photo_shelf(type)
        elif choice == "b":
            controller.test_photo_box()
        elif choice == "c":
            type = input("请输入类型（1-4）：")
            if type != 1 and type != 2 and type != 3 and type != 4:
                print("输入错误，请重新输入")
                continue
            controller.test_grasp(type)
        elif choice == "t":
            type = input("请输入类型（1-4）：")
            if type != 1 and type != 2 and type != 3 and type != 4:
                print("输入错误，请重新输入")
                continue
            controller.test_throw(type)
        elif choice == "n":
            poseKey = raw_input("请输入导航位置：")
            if poseKey not in controller.position:
                print("输入错误，请重新输入")
                continue
            controller.navigate_posekey(poseKey)
        elif choice == "z":
            theta = raw_input("请输入自转角度：")
            if not theta.isdigit():
                print("输入错误，请重新输入")
                continue
            theta = float(theta)
            controller.BM.moveRotate(theta)

        elif choice == "p":
            poseKey = raw_input("请输入校准位置：")
            if poseKey not in controller.position:
                print("输入错误，请重新输入")
                continue
            controller.calibratePose(poseKey)
        elif choice == "l":
            controller.position = controller.loadToDict(position_path, mode="pose")
        elif choice == "q":
            break

        #下 下 1
        #下 上 2
        #上 左 3
        #下 左 1