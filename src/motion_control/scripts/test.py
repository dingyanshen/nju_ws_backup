#! /usr/bin/env python2.7
# -*- coding:utf-8 -*-

import rospy
import json
import actionlib
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion
from actionlib_msgs.msg import *
from camera.srv import PhotoshelfService, PhotoboxService, PhotoService
from dobot.srv import GraspService, ThrowService
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from basic_move import BasicMove

class NavigationRule:
    @staticmethod
    def _match_rule(current, target, nav_rules): # 匹配当前位置到目标位置的导航逻辑
        if current in nav_rules:
            current_rules = nav_rules[current]
            for target_group, rule in current_rules.items():
                if any(NavigationRule._is_in_group(target, g) for g in target_group.split(',')):
                    return rule
            if '*' in current_rules:
                return current_rules['*']
        return nav_rules.get('default', {})
    
    @staticmethod
    def _is_in_group(target, group): # 判断目标所属组
        # 处理通配符
        if group == '*':
            return True
        # 处理单元素组
        return target == group

    @staticmethod
    def _execute_pre(pre_actions, BM): # 执行预处理动作
        for action in pre_actions:
            if action[0] == 'rotate':
                BM.moveRotate(action[1])
            elif action[0] == 'forward':
                BM.moveForward(action[1])

    @staticmethod
    def _navigate_posekey_waypoint(poseKey, move_base_AS, position): # 导航到中间点的低精度模式
        # 获取原来的参数
        original_xy_tolerance = rospy.get_param("/move_base/TebLocalPlannerROS/xy_goal_tolerance")
        original_yaw_tolerance = rospy.get_param("/move_base/TebLocalPlannerROS/yaw_goal_tolerance")
        
        try:
            rospy.set_param("/move_base/TebLocalPlannerROS/xy_goal_tolerance", 0.05)
            rospy.set_param("/move_base/TebLocalPlannerROS/yaw_goal_tolerance", 0.05)

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = position[poseKey]
            move_base_AS.send_goal(goal)
            while not move_base_AS.wait_for_result():
                pass
                
        finally:
            rospy.set_param("/move_base/TebLocalPlannerROS/xy_goal_tolerance", original_xy_tolerance)
            rospy.set_param("/move_base/TebLocalPlannerROS/yaw_goal_tolerance", original_yaw_tolerance)

class MainController:
    def __init__(self, position_path, navigate_path):
        rospy.init_node('main_controller')
        self.BM = BasicMove(detailInfo=True)
        self.position = self.loadToDict(position_path, mode="pose")
        self.nav_rules = self.loadToDict(navigate_path, mode="nav")
        self.move_base_AS = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base_AS.wait_for_server(rospy.Duration(60))
        self.pub_initialpose = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=20)
        rospy.wait_for_service('photo_shelf_service')
        rospy.wait_for_service('photo_box_service')
        rospy.wait_for_service('photo_service')
        rospy.wait_for_service('dobot_grasp_service')
        rospy.wait_for_service('dobot_throw_service')
        self.photo_shelf_proxy = rospy.ServiceProxy('photo_shelf_service', PhotoshelfService)
        self.photo_box_proxy = rospy.ServiceProxy('photo_box_service', PhotoboxService)
        self.photo_proxy = rospy.ServiceProxy('photo_service', PhotoService)
        self.grasp_proxy = rospy.ServiceProxy('dobot_grasp_service', GraspService)
        self.throw_proxy = rospy.ServiceProxy('dobot_throw_service', ThrowService)

        self.CURRENT_STATE = "INIT" # INIT 初始化 PHOTO 拍照 RUN 运行
        self.CURRENT_LOCATION = "INIT" # INIT 初始化 poseKey 坐标

    def loadToDict(self, file_path, mode): # 导入相关参数
        if mode == "pose":
            dictionary = dict()
            file = open(file_path, "r")
            for line in file:
                data = line.strip().split()
                if len(data) == 5:
                    key, px, py, qz, qw = data
                    px, py, qz, qw = float(px), float(py), float(qz), float(qw)
                    pose = Pose(Point(px, py, 0.0), Quaternion(0.0, 0.0, qz, qw))
                    dictionary[key] = pose
            file.close()
            print("成功导入坐标参数！")
            return dictionary
        elif mode == "nav":
            with open(file_path, 'r') as f:
                return json.load(f)

    def calibratePose(self, poseKey): # 校准位姿
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
        print("成功校准位姿为 " + str(poseKey) + "!")

    def navigate_posekey(self, poseKey): # 导航到指定位置函数外层
        # 初始化或拍照状态下的逻辑
        if self.CURRENT_STATE in ["INIT", "PHOTO"]:
            self._navigate_posekey(poseKey)
            self.CURRENT_LOCATION = poseKey
            return
        
        # 运行状态下的逻辑
        if self.CURRENT_STATE == "RUN":
            # 当前位置已经是目标位置
            if self.CURRENT_LOCATION == poseKey:
                return
            # 匹配导航规则
            rule = NavigationRule._match_rule(self.CURRENT_LOCATION, poseKey, self.nav_rules)
            # 执行预处理
            NavigationRule._execute_pre(rule.get('pre', []), self.BM)
            # 执行中间点
            for wp in rule.get('waypoints', []):
                NavigationRule._navigate_posekey_waypoint(wp, self.move_base_AS, self.position)
            self._navigate_posekey(poseKey)
            self.CURRENT_LOCATION = poseKey

    def _navigate_posekey(self, poseKey): # 导航到指定位置
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
        print(response.result)

    def test_grasp(self, type, up):
        response = self.photo_proxy(type)
        if up == 1 and type <= 10:
            catch_type = [1, 1, response.error_x, response.error_y]
        elif up == 1 and type > 10:
            catch_type = [0, 1, response.error_x, response.error_y]
        elif up == 0 and type <= 10:
            catch_type = [1, 0, response.error_x, response.error_y]
        elif up == 0 and type > 10:
            catch_type = [0, 0, response.error_x, response.error_y]
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
    navigate_path = "/home/eaibot/nju_ws/src/motion_control/config/nav_rules.json"
    controller = MainController(position_path, navigate_path)
    controller.calibratePose("start_left")
    controller.CURRENT_LOCATION = "start_left"

    while True:
        choice = raw_input("——————————————————————————————————————————————————————————————————————————————————\n" +
                           "请输入选择 m状态 s拍货架 b拍邮箱 c抓取 t投递 n导航 z自转 p校准 l加载 q退出：")
        if choice == "m":
            print("当前状态：" + controller.CURRENT_STATE)
            print("当前位置：" + controller.CURRENT_LOCATION)
            modi = raw_input("请输入新状态（1-INIT 2-PHOTO 3-RUN）：")
            if modi == "1":
                controller.CURRENT_STATE = "INIT"
            elif modi == "2":
                controller.CURRENT_STATE = "PHOTO"
            elif modi == "3":
                controller.CURRENT_STATE = "RUN"
            else:
                print("输入错误，请重新输入")
                continue
        elif choice == "s":
            type = raw_input("请输入类型（1-6）：")
            if type != "1" and type != "2" and type != "3" and type != "4" and type != "5" and type != "6":
                print("输入错误，请重新输入")
                continue
            controller.test_photo_shelf(int(type))
        elif choice == "b":
            controller.test_photo_box()
        elif choice == "c":
            type = raw_input("请输入抓取类型（1-20）：")
            if not type.isdigit() or int(type) < 1 or int(type) > 20:
                print("输入错误，请重新输入")
                continue
            up = raw_input("请输入抓取高度（0-下 1-上）：")
            if up != "0" and up != "1":
                print("输入错误，请重新输入")
                continue
            type = int(type)
            up = int(up)
            controller.test_grasp(type, up)
        elif choice == "t":
            type = raw_input("请输入投递类型（1-4）：")
            if type != "1" and type != "2" and type != "3" and type != "4":
                print("输入错误，请重新输入")
                continue
            controller.test_throw(int(type))
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
            controller.nav_rules = controller.loadToDict(navigate_path, mode="nav")
        elif choice == "q":
            break