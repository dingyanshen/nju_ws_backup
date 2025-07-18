#! /usr/bin/env python2.7
# -*- coding:utf-8 -*-

import os
import pickle
import codecs
import rospy
import json
import random
import actionlib
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion
from actionlib_msgs.msg import *
from camera.srv import PhotoshelfService, PhotoboxService, PhotoService
from dobot.srv import GraspService, ThrowService
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from basic_move import BasicMove

class StateManager: # 救援模式状态管理类
    def __init__(self, save_path, log_path):
        self.save_path = save_path
        self.log_path = log_path
        self.state_data = {
            "completed_ids": set(), # 已完成的邮件编号
            "current_phase": "INIT", # 用于救援模式的当前阶段
            "grabbed_mails": [], # 已抓取但未投递的邮件

            "mail_table": [],  
            "mail_table_L": [],  
            "mail_table_R": [],  
            "mail_box": [],  
            "L_provinces": [],  
            "R_provinces": [],   
            "priority_provinces": [], 
            "success_provinces": set(), 
            "failed_boxes": [] 
        }

    def save_state(self, controller): # 保存当前状态到文件
        self.state_data["completed_ids"] = controller.completed_ids
        self.state_data["current_phase"] = controller.CURRENT_PHASE
        self.state_data["grabbed_mails"] = controller.grabbed_mails

        self.state_data["mail_table"] = controller.mail_table
        self.state_data["mail_table_L"] = controller.mail_table_L
        self.state_data["mail_table_R"] = controller.mail_table_R
        self.state_data["mail_box"] = controller.mail_box
        self.state_data["L_provinces"] = controller.L_provinces
        self.state_data["R_provinces"] = controller.R_provinces
        self.state_data["priority_provinces"] = controller.priority_provinces
        self.state_data["success_provinces"] = controller.success_provinces
        self.state_data["failed_boxes"] = controller.failed_boxes

        with open(self.save_path, "wb") as f:
            pickle.dump(self.state_data, f)
        self._write_log(controller)
                        
    def _write_log(self, controller): # 写入包含详细信息的日志
        log_content = []
        log_content.append("PHASE:" + controller.CURRENT_PHASE)
        log_content.append("")

        log_content.append("SUCCESS_PRO:" + str(len(controller.success_provinces)))
        log_content.append("FAILED_PRO:" + str(len(controller.failed_boxes)))
        log_content.append("")

        log_content.append("COMPLETED_MAILS:" + str(len(controller.completed_ids)))
        log_content.append("GRABBED_MAILS:" + str(len(controller.grabbed_mails)))
        log_content.append("")

        log_content.append("MAILS TABLE:")
        if controller.mail_table:
            for mail in controller.mail_table:
                mail_id = controller.get_mail_id(mail)
                if mail_id in controller.completed_ids:
                    status = "DONE"
                else:
                    status = "PENDING"
                log_content.append("NO." + str(mail_id) + " " + str(mail['results']) + " " + status)
        else:
            log_content.append("NO MAILS FOUND")
        log_content.append("")

        log_content.append("BOXES TABLE:")
        if controller.mail_box:
            for box in controller.mail_box:
                log_content.append("ID." + str(box['box_id']) + " " + str(box['result']))
        else:
            log_content.append("NO BOXES FOUND")
        log_content.append("")

        with codecs.open(self.log_path, "w", encoding="utf-8") as f:
            f.write("\n".join(log_content))

    def load_state(self): # 加载之前保存的状态
        if os.path.exists(self.save_path):
            with open(self.save_path, "rb") as f:
                self.state_data = pickle.load(f)
            print("已从" + self.save_path + "加载历史状态")
            return self.state_data
        return None

    def clear_state(self): # 清除之前保存的状态
        if os.path.exists(self.save_path):
            os.remove(self.save_path)
        print("已清除历史状态文件")

class NavigationRule: # 导航规则类
    @staticmethod
    def _match_rule(current, target, nav_rules): # 匹配当前位置到目标位置的导航逻辑
        if current in nav_rules:
            current_rules = nav_rules[current]
            for target_group, rule in current_rules.items():
                if target_group != 'default' and target in target_group.split(','):
                    return rule
            if 'default' in current_rules:
                return current_rules['default']
        return nav_rules.get('default', {})

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
            rospy.set_param("/move_base/TebLocalPlannerROS/xy_goal_tolerance", 0.08)
            rospy.set_param("/move_base/TebLocalPlannerROS/yaw_goal_tolerance", 0.08)

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

class MainController: # 主控制器类
    def __init__(self, position_path, navigate_path, state_save_path, log_path):
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

        self.mail_table = [] # results.positions_z.positions_x
        self.mail_table_L = [] # 左侧货架
        self.mail_table_R = [] # 右侧货架
        self.mail_box = [] # box_id.result
        self.priority_provinces = [] # 省份优先级
        self.L_provinces = [] # 左侧省份
        self.R_provinces = [] # 右侧省份
        self.platform_state = 0 # 平台状态

        self.expected_provinces = set(range(1, 9))  # 预期省份编号为1 2 3 4 5 6 7 8
        self.failed_boxes = []  # 识别失败的箱子编号
        self.success_provinces = set()  # 识别成功的省份编号

        self.CURRENT_STATE = "INIT" # INIT 初始化 PHOTO 拍照 RUN 运行
        self.CURRENT_LOCATION = "INIT" # INIT 初始化 poseKey 坐标

        self.CURRENT_PHASE = "PHOTO" # 用于救援模式的当前阶段
        self.grabbed_mails = [] # 已抓取但未投递的邮件
        self.completed_ids = set()  # 已完成的邮件编号
        self.state_manager = StateManager(state_save_path, log_path) # 状态管理器实例
        self.resume_mode = False # 是否为救援模式
        self.check_rescue_mode() # 检查救援模式

    def check_rescue_mode(self): # 检查救援模式
        # 检查状态文件是否存在
        if os.path.exists(self.state_manager.save_path):
            while True:
                choice = raw_input("检测到上次异常退出的状态数据，输入H进入救援模式，输入C删除状态数据：").lower()
                if choice == 'h':  # 进入救援模式并加载状态
                    self.resume_mode = True
                    loaded_state = self.state_manager.load_state()
                    if loaded_state:
                        self.CURRENT_PHASE = loaded_state["current_phase"]
                        self.completed_ids = loaded_state["completed_ids"]
                        self.grabbed_mails = loaded_state["grabbed_mails"]
                        self.mail_table = loaded_state["mail_table"]
                        self.mail_table_L = loaded_state["mail_table_L"]
                        self.mail_table_R = loaded_state["mail_table_R"]
                        self.mail_box = loaded_state["mail_box"]
                        self.L_provinces = loaded_state["L_provinces"]
                        self.R_provinces = loaded_state["R_provinces"]
                        self.priority_provinces = loaded_state["priority_provinces"]
                        self.success_provinces = loaded_state["success_provinces"]
                        self.failed_boxes = loaded_state["failed_boxes"]
                        print("已进入救援模式 状态：" + str(self.CURRENT_PHASE) + " 已完成：" + str(len(self.completed_ids)) + " 平台上剩余：" + str(len(self.grabbed_mails)) + "!")
                    else:
                        print("状态数据加载失败，将从头开始新任务！")
                        self.resume_mode = False
                    break  # 退出循环，完成处理
                elif choice == 'c':  # 清除状态数据
                    self.state_manager.clear_state()
                    print("已清除历史状态数据，将从头开始新任务！")
                    break  # 退出循环，完成处理
                else:
                    pass
        else:
            print("未检测到历史状态数据，将从头开始新任务！")

    def get_mail_id(self, mail): # 计算邮件唯一编号
        # 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20
        if mail['positions_z'] == 1:
            return mail['positions_x']
        else:
            return 10 + mail['positions_x']

    def AutoSet(self): # 模拟拍照阶段 随机邮箱和邮件设置
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
        # 按下 ENTER 开始跑车
        raw_input("按下 ENTER 开始跑车...")

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
        print("到达右上-左拍照点位!")
        self.process_box("RU2")

        self.navigate_posekey("ARU1")
        print("到达右上-右拍照点位!")
        self.process_box("RU1")
        self.BM.moveForward(-0.3)

    def takeboxPic_RD(self): # 邮箱文字识别[右下]
        self.navigate_posekey("ARD2")
        print("到达右下-下拍照点位!")
        self.process_box("RD2")

        self.navigate_posekey("ARD1")
        print("到达右下-上拍照点位!")
        self.process_box("RD1")
        
    def takeboxPic_LU(self): # 邮箱文字识别[左上]
        self.navigate_posekey("ALU1")
        print("到达左上-左拍照点位!")
        self.process_box("LU1")

        self.navigate_posekey("ALU2")
        print("到达左上-右拍照点位!")
        self.process_box("LU2")

    def takeboxPic_LD(self): # 邮箱文字识别[左下]
        self.navigate_posekey("ALD1")
        print("到达左下-上拍照点位!")
        self.process_box("LD1")

        self.navigate_posekey("ALD2")
        print("到达左下-下拍照点位!")
        self.process_box("LD2")
                
    def takeshelfPic_R(self): # 货架拍照[右侧]
        self.navigate_posekey("RP3")
        print("到达右侧上拍照RP3点位!")
        self.process_shelf(4)
        
        self.navigate_posekey("RP2")
        print("到达右侧中拍照RP2点位!")
        self.process_shelf(5)
        
        self.navigate_posekey("RP1")
        print("到达右侧下拍照RP1点位!")
        self.process_shelf(6)

    def takeshelfPic_L(self): # 货架拍照[左侧]
        self.navigate_posekey("LP3")
        print("到达左侧下拍照LP3点位!")
        self.process_shelf(1)

        self.navigate_posekey("LP2")
        print("到达左侧中拍照LP2点位!")
        self.process_shelf(2)
        
        self.navigate_posekey("LP1")
        print("到达左侧上拍照LP1点位!")
        self.process_shelf(3)

    def process_shelf(self, type): # 货架拍照服务
        try:
            response = self.photo_shelf_proxy(type)
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
            print("货架拍照服务调用失败!")
        self.state_manager.save_state(self)

    def process_box(self, box_id): # 邮箱拍照服务
        try:
            response = self.photo_box_proxy()
            result = response.result
            if result in self.expected_provinces and result not in self.success_provinces:
                self.mail_box.append({
                        'box_id': box_id,
                        'result': result
                    })
                self.priority_provinces.append(result)
                if box_id == "LU1" or box_id == "LU2" or box_id == "LD1" or box_id == "LD2":
                    self.L_provinces.append(response.result)
                elif box_id == "RU1" or box_id == "RU2" or box_id == "RD1" or box_id == "RD2":
                    self.R_provinces.append(response.result)
                self.success_provinces.add(result)
            else:
                self.failed_boxes.append(box_id)
                print("邮箱识别无效或重复!")
        except rospy.ServiceException as e:
            print("邮箱拍照服务调用失败!")
        self.state_manager.save_state(self)

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
                print("抓取邮件失败！")
                return False
            if response.success:
                return True
            else:
                print("抓取邮件失败！")
                return False
        except rospy.ServiceException as e:
            print("抓取服务调用失败！")
            return False
        
    def deliver_mails(self, mails): # 运送邮件
        mails.reverse()
        try:
            for mail in mails:
                found = False
                for box in self.mail_box:
                    if box['result'] == mail['results']:
                        print("省份编号" + str(mail['results']) + "的邮件正在运送到邮箱" + str(box['box_id']))
                        self.navigate_posekey(box['box_id'])
                        found = True
                        break
                if not found:
                    print("未找到对应的邮箱，无法运送省份编号" + str(mail['results']) + "的邮件")
                    print("省份编号" + str(mail['results']) + "的邮件将被丢弃到无效邮箱")
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
                    print("运送邮件失败！")
                    return False
            return True
        except rospy.ServiceException as e:
            print("运送服务调用失败！")
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
            mail_id = self.get_mail_id(mail)
            if mail_id in self.completed_ids:  # 如果邮件已经完成，跳过
                continue

            if mail['results'] not in priority_provinces:
                break

            if self.grasp_mail(mail):
                grabbed_mails.append(mail)

                if len(grabbed_mails) >= 2:
                    if self.deliver_mails(grabbed_mails):
                        for m in grabbed_mails:
                            self.completed_ids.add(self.get_mail_id(m))
                        self.stage_manager.save_state(self) # 状态记录
                    grabbed_mails = []

        if grabbed_mails:
            if self.deliver_mails(grabbed_mails):
                for m in grabbed_mails:
                    self.completed_ids.add(self.get_mail_id(m))
                self.stage_manager.save_state(self) # 状态记录

        return [m for m in mail_table if self.get_mail_id(m) not in self.completed_ids]

    def process_non_priority_mails(self, mail_table): # 处理非优先省份邮件
        grabbed_mails = []
        for mail in mail_table:
            mail_id = self.get_mail_id(mail)
            if mail_id in self.completed_ids:  # 如果邮件已经完成，跳过
                continue

            if self.grasp_mail(mail):
                grabbed_mails.append(mail)

                if len(grabbed_mails) >= 2:
                    if self.deliver_mails(grabbed_mails):
                        for m in grabbed_mails:
                            self.completed_ids.add(self.get_mail_id(m))
                        self.state_manager.save_state(self) # 状态记录
                    grabbed_mails = []

        if grabbed_mails:
            if self.deliver_mails(grabbed_mails):
                for m in grabbed_mails:
                    self.completed_ids.add(self.get_mail_id(m))
                self.state_manager.save_state(self) # 状态记录

    def handle_failed_boxes(self): # 处理失败的邮箱
        missing_provinces = self.expected_provinces - self.success_provinces
        missing_list = list(missing_provinces)
        random.shuffle(missing_list)
        for box_id, province in zip(self.failed_boxes, missing_list):
            self.mail_box.append({'box_id': box_id, 'result': province})
            self.priority_provinces.append(province)
            if box_id in ["LU1", "LU2", "LD1", "LD2"]:
                self.L_provinces.append(province)
            else:
                self.R_provinces.append(province)
            self.success_provinces.add(province)
            print("失败的邮箱" + str(box_id) + "已被重新设置为省份编号" + str(province) + "!")

    def run(self):
        self.welcome() # 欢迎界面 按下ENTER开始跑车

        self.calibratePose("start_left") # 校准起始位姿

        self.CURRENT_LOCATION = "start_left"

        self.mail_box.append({'box_id': "U",'result': 0}) # 无效箱子
        self.priority_provinces.append(0) # 无效箱子优先
        self.L_provinces.append(0) # 无效箱子优先
        self.R_provinces.append(0) # 无效箱子优先

        self.CURRENT_STATE = "PHOTO"

        if not self.resume_mode: # 救援模式不需要拍照
            self.takeboxPic_LD() # 邮箱拍照[左下]
            self.takeshelfPic_L() # 货架拍照[左侧]
            self.takeboxPic_LU() # 邮箱拍照[左上]
            self.takeboxPic_RU() # 邮箱拍照[右上]
            self.takeshelfPic_R() # 货架拍照[右侧]
            self.takeboxPic_RD() # 邮箱拍照[右下]
            self.handle_failed_boxes() # 处理失败的邮箱
        else:
            self.CURRENT_STATE = "RUN"
            print("救援模式已启用，跳过拍照阶段！")

        self.navigate_posekey("start")
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

        self.state_manager.clear_state() # 清除之前保存的状态
        self.end() # 结束界面

if __name__ == "__main__":
    position_path = "/home/eaibot/nju_ws/src/motion_control/config/position.txt"
    navigate_path = "/home/eaibot/nju_ws/src/motion_control/config/nav_rules.json"
    state_save_path = "/home/eaibot/nju_ws/src/motion_control/config/state_save.pkl"
    log_path = "/home/eaibot/nju_ws/src/motion_control/config/state_log.txt"
    controller = MainController(position_path, navigate_path, state_save_path, log_path) # 创建主控制器实例
    controller.run() # 启动主控制器