#! /usr/bin/env python2.7
# -*- coding:utf-8 -*-

import os
import pickle
import codecs
import rospy
import json
import random
import actionlib
from collections import defaultdict
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
            "grabbed_mails": [], # 已抓取但未投递的邮件
            "mail_table": [], # 邮件静态字典
            "mail_box": [], # 邮箱静态字典
            "platform_state": 0, # 平台状态
        }

    def save_state(self, controller): # 保存当前状态到文件
        self.state_data["completed_ids"] = controller.completed_ids
        self.state_data["grabbed_mails"] = controller.grabbed_mails
        self.state_data["mail_table"] = controller.mail_table
        self.state_data["mail_box"] = controller.mail_box
        self.state_data["platform_state"] = controller.platform_state

        with open(self.save_path, "wb") as f:
            pickle.dump(self.state_data, f)
        self._write_log(controller)
                        
    def _write_log(self, controller): # 写入包含详细信息的日志
        log_content = []

        log_content.append("COMPLETED_MAILS:" + str(len(controller.completed_ids)))
        log_content.append("GRABBED_MAILS:" + str(len(controller.grabbed_mails)))
        log_content.append("PLATFORM_STATE:" + str(controller.platform_state))
        log_content.append("")

        # 打印邮件静态字典
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

        # 打印邮箱静态字典
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

class MainController: # 基础功能类
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

        # 邮件静态字典 {'results': _, 'positions_z': _, 'positions_x': _, 'side': _}
        self.mail_table = []

        # 邮箱静态字典 {'box_id': _, 'result': _, 'side': _}
        self.mail_box = []

        # 以下用于处理失败的邮箱
        self.expected_provinces = set(range(1, 9))  # 预期省份编号为 1 2 3 4 5 6 7 8
        self.failed_boxes = []  # 识别失败的箱子编号
        self.success_provinces = set()  # 识别成功的省份编号

        self.platform_state = 0 # 平台状态
        self.CURRENT_STATE = "INIT" # INIT 初始化 PHOTO 拍照 RUN 运行
        self.CURRENT_LOCATION = "INIT" # INIT 初始化 poseKey 坐标
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
                        self.mail_table = loaded_state["mail_table"]
                        self.mail_box = loaded_state["mail_box"]
                        self.completed_ids = loaded_state["completed_ids"]
                        self.grabbed_mails = loaded_state["grabbed_mails"]
                        self.platform_state = loaded_state["platform_state"]
                        print("已进入救援模式 已完成：" + str(len(self.completed_ids)) + " 平台上剩余：" + str(len(self.grabbed_mails)) + "!")
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
                self._navigate_posekey(poseKey) # 无预处理和中间点
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

    def get_mail_id(self, mail): # 计算邮件唯一编号
        # 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20
        if mail['positions_z'] == 1:
            return mail['positions_x']
        else:
            return 10 + mail['positions_x']

class MainController(MainController): # 拍照服务类
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
                        'side': 'L',
                    })
            elif type == 5 or type == 6:
                for i in range(4):
                    self.mail_table.append({
                        'results': response.results[i],
                        'positions_z': response.positions_z[i],
                        'positions_x': response.positions_x[i],
                        'side': 'R',
                    })
            elif type == 3:
                self.mail_table.append({
                    'results': response.results[0],
                    'positions_z': response.positions_z[0],
                    'positions_x': response.positions_x[0],
                    'side': 'L',
                })
                self.mail_table.append({
                    'results': response.results[2],
                    'positions_z': response.positions_z[2],
                    'positions_x': response.positions_x[2],
                    'side': 'L',
                })
            elif type == 4:
                self.mail_table.append({
                    'results': response.results[1],
                    'positions_z': response.positions_z[1],
                    'positions_x': response.positions_x[1],
                    'side': 'R',
                })
                self.mail_table.append({
                    'results': response.results[3],
                    'positions_z': response.positions_z[3],
                    'positions_x': response.positions_x[3],
                    'side': 'R',
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
                        'result': result,
                        'side': 'L' if box_id.startswith('L') else 'R',
                    })
                self.success_provinces.add(result)
            else:
                self.failed_boxes.append(box_id)
                print("邮箱识别无效或重复!")
        except rospy.ServiceException as e:
            print("邮箱拍照服务调用失败!")
        self.state_manager.save_state(self)

    def handle_failed_boxes(self): # 处理失败的邮箱
        missing_provinces = self.expected_provinces - self.success_provinces
        missing_list = list(missing_provinces)
        random.shuffle(missing_list)
        for box_id, province in zip(self.failed_boxes, missing_list):
            self.mail_box.append({'box_id': box_id, 'result': province, 'side': 'L' if box_id.startswith('L') else 'R'})
            self.success_provinces.add(province)
            print("失败的邮箱" + str(box_id) + "已被重新设置为省份编号" + str(province) + "!")

class MainController(MainController): # 邮件处理类
    def sort_mails_by_pairs(self, mails): # 对邮件列表重排序 邮件对在前邮件单在后
        # 将邮件按省份分组
        groups = defaultdict(list)
        for mail in mails:
            code = mail['results']
            groups[code].append(mail)
        
        # 每个组都有邮件对和邮件单
        paired_parts = []  # 邮件对
        single_parts = []  # 邮件单

        # 分开每个组的邮件对和邮件单
        for code, mails in groups.items():
            count = len(mails)
            pair_count = count // 2 # 邮件对数目
            total_paired = pair_count * 2 # 邮件对的邮件数
            paired_parts.extend(mails[:total_paired]) # 邮件对
            if count % 2 == 1: single_parts.extend(mails[total_paired:]) # 邮件单
        return paired_parts + single_parts

    def select_side(self, side): # 获得侧邮件 并根据邮件对邮件单排序
        # 筛选side侧邮件
        sidemails = [mail for mail in self.mail_table if mail['side'] == side]
        # 筛选side侧和U侧邮箱
        sideboxes = [box for box in self.mail_box if box['side'] == side or box['side'] == 'U']
        # side侧邮件中属于sideboxes的邮件优先
        side_provinces = [box['result'] for box in sideboxes]
        on_side_mails = [mail for mail in sidemails if mail['results'] in side_provinces]
        # 对邮件进行排序 邮件对在前邮件单在后
        on_side_mails = self.sort_mails_by_pairs(on_side_mails)
        return on_side_mails

    def select_non_side(self, side): # 获得跨邮件 side指货架侧 并根据邮件对邮件单排序
        # 筛选side侧邮件
        sidemails = [mail for mail in self.mail_table if mail['side'] == side]
        # 筛选non_side侧邮箱
        non_sideboxes = [box for box in self.mail_box if box['side'] != side and box['side'] != 'U']
        # side侧邮件中属于non_sideboxes的邮件优先
        non_side_provinces = [box['result'] for box in non_sideboxes]
        non_side_mails = [mail for mail in sidemails if mail['results'] in non_side_provinces]
        # 对邮件进行排序 邮件对在前邮件单在后
        non_side_mails = self.sort_mails_by_pairs(non_side_mails)
        return non_side_mails

    def grasp_mail(self, mail): # 抓取单个邮件
        self.state_manager.save_state(self) # 状态记录
        try:
            # 导航到抓取点位
            if mail['positions_z'] == 1: # 上层邮件
                self.navigate_posekey("CL" + str(mail['positions_x']) + "_UP")
            elif mail['positions_z'] == 2: # 下层邮件
                self.navigate_posekey("CL" + str(mail['positions_x']))
            # 执行抓取
            if mail['positions_z'] == 1 and self.platform_state == 0: # 上层抓到下层
                response = self.photo_proxy(mail['positions_x'])
                if not self.is_safe_error(response.error_x, response.error_y):
                    print("抓取邮件时误差超出安全范围！")
                    self.retry_navigate_to_shelf("CL" + str(mail['positions_x']) + "_UP")
                    response = self.photo_proxy(mail['positions_x'])
                    if not self.is_safe_error(response.error_x, response.error_y):
                        print("抓取邮件时误差仍超出安全范围，放弃抓取！")
                        return False
                catch_type = [1, 0, response.error_x, response.error_y]
                response = self.grasp_proxy(*catch_type)
            elif mail['positions_z'] == 1 and self.platform_state == 1: # 上层抓到上层
                response = self.photo_proxy(mail['positions_x'])
                if not self.is_safe_error(response.error_x, response.error_y):
                    print("抓取邮件时误差超出安全范围！")
                    self.retry_navigate_to_shelf("CL" + str(mail['positions_x']) + "_UP")
                    response = self.photo_proxy(mail['positions_x'])
                    if not self.is_safe_error(response.error_x, response.error_y):
                        print("抓取邮件时误差仍超出安全范围，放弃抓取！")
                        return False
                catch_type = [1, 1, response.error_x, response.error_y]
                response = self.grasp_proxy(*catch_type)
            elif mail['positions_z'] == 2 and self.platform_state == 0: # 下层抓到下层
                response = self.photo_proxy(mail['positions_x']+10)
                if not self.is_safe_error(response.error_x, response.error_y):
                    print("抓取邮件时误差超出安全范围！")
                    self.retry_navigate_to_shelf("CL" + str(mail['positions_x']))
                    response = self.photo_proxy(mail['positions_x']+10)
                    if not self.is_safe_error(response.error_x, response.error_y):
                        print("抓取邮件时误差仍超出安全范围，放弃抓取！")
                        return False
                catch_type = [0, 0, response.error_x, response.error_y]
                response = self.grasp_proxy(*catch_type)
            elif mail['positions_z'] == 2 and self.platform_state == 1: # 下层抓到上层
                response = self.photo_proxy(mail['positions_x']+10)
                if not self.is_safe_error(response.error_x, response.error_y):
                    print("抓取邮件时误差超出安全范围！")
                    self.retry_navigate_to_shelf("CL" + str(mail['positions_x']))
                    response = self.photo_proxy(mail['positions_x']+10)
                    if not self.is_safe_error(response.error_x, response.error_y):
                        print("抓取邮件时误差仍超出安全范围，放弃抓取！")
                        return False
                catch_type = [0, 1, response.error_x, response.error_y]
                response = self.grasp_proxy(*catch_type)
            else:
                print("抓取邮件时平台状态异常！")
                return False
            # 检查抓取结果 更新平台状态
            if response.success:
                self.platform_state += 1 # 平台状态更新
                self.grabbed_mails.append(mail) # 添加到已抓取邮件列表
                self.state_manager.save_state(self) # 状态记录
                return True
            else:
                print("抓取邮件失败！")
                return False
        except rospy.ServiceException as e:
            print("抓取服务调用失败！")
            return False

    def throw_mail(self, mail): # 投递单个邮件
        self.state_manager.save_state(self) # 状态记录
        try:
            # 导航到投递点位
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
            # 执行投递
            if self.platform_state == 2: # 上层
                throw_type = [1, 0]
                response = self.throw_proxy(*throw_type)
            elif self.platform_state == 1: # 下层
                throw_type = [0, 0]
                response = self.throw_proxy(*throw_type)
            else:
                print("投递邮件时平台状态异常！")
                return False
            # 检查投递结果 更新平台状态
            if response.success:
                self.platform_state -= 1
                self.grabbed_mails.pop(-1) # 从已抓取邮件列表中移除
                self.completed_ids.add(self.get_mail_id(mail)) # 添加到已完成邮件编号集合
                self.state_manager.save_state(self) # 状态记录
                return True
            else:
                print("投递邮件失败！")
                return False
        except rospy.ServiceException as e:
            print("投递服务调用失败！")
            return False

    def process_mails(self, mails): # 处理同区邮件
        if not mails:  # 如果没有邮件直接返回
            return
        
        remains = [] # 剩余邮件列表
        
        index = 0
        while index < len(mails):
            mail = mails[index] # 按顺序处理
            if self.grasp_mail(mail): # 抓取邮件成功
                if len(self.grabbed_mails) == 2:
                    self.throw_mail(self.grabbed_mails[-1]) # 投递上层
                    self.throw_mail(self.grabbed_mails[-1]) # 投递下层
                index += 1
            else: # 抓取失败
                index += 1
                if index >= len(mails): # 不存在下一个邮件
                    break
                else: # 存在下一个邮件
                    next_mail = mails[index]
                    remains.append(next_mail) # 将下一个邮件添加到剩余列表
                    index += 1 # 跳过下一个邮件

        if self.grabbed_mails:
            self.throw_mail(self.grabbed_mails[-1]) # 投递下层

        self.process_mails(remains) # 递归处理剩余邮件

    def process_non_side_mails(self, shelf_R, shelf_L): # 处理跨区邮件
        if not shelf_R and not shelf_L:  # 如果没有跨区邮件直接返回
            return

        while shelf_L or shelf_R: # 交替抓取
            if shelf_L: # 左侧货架
                left_mails = shelf_L[:2]
                for mail in left_mails:
                    self.grasp_mail(mail)
                    shelf_L.pop(0)
                while self.grabbed_mails:
                    self.throw_mail(self.grabbed_mails[-1])
            
            if shelf_R: # 右侧货架
                right_mails = shelf_R[:2]
                for mail in right_mails:
                    self.grasp_mail(mail)
                    shelf_R.pop(0)
                while self.grabbed_mails:
                    self.throw_mail(self.grabbed_mails[-1])

    def resume_process_mails(self): # 恢复处理未完成的邮件
        if not self.grabbed_mails: # 没有剩余邮件
            self.platform_state = 0  # 重置平台状态
            return
        print("恢复处理未完成的邮件...")
        # 处理已抓取但未投递的邮件
        while self.grabbed_mails:
            mail = self.grabbed_mails[-1]
            if self.platform_state == 2:
                if self.throw_mail(mail):
                    print("已投递上层邮件 " + str(self.get_mail_id(mail)))
                else:
                    break
            elif self.platform_state == 1:
                if self.throw_mail(mail):
                    print("已投递下层邮件 " + str(self.get_mail_id(mail)))
                else:
                    break
            else:
                print("平台状态异常，无法投递邮件 " + str(self.get_mail_id(mail)))
                break
        # 所有邮件投递完成后重置平台状态
        if not self.grabbed_mails:
            self.platform_state = 0

    def delete_completed_mails(self, mails): # 删除已完成的邮件并重排
        # 删除已完成的邮件
        completed_ids = self.completed_ids
        new_mails = [mail for mail in mails if self.get_mail_id(mail) not in completed_ids]
        # 重排邮件
        new_mails = self.sort_mails_by_pairs(new_mails)
        return new_mails

    def is_safe_error(self, error_x, error_y): # 判断误差是否在安全范围内
        if error_x == error_y == 0: # 非真实误差也是不安全的
            return False
        # TODO: 如果误差超过安全范围则返回False
        return True

    def retry_navigate_to_shelf(self, posekey): # 重新尝试导航货架
        if posekey in ["CL1", "CL2", "CL3", "CL4", "CL5", "CL1_UP", "CL2_UP", "CL3_UP", "CL4_UP", "CL5_UP"]:
            self.BM.moveRotate(120) # 旋转到120度
            self.BM.moveForward(-0.4) # 后退0.4米
        elif posekey in ["CL6", "CL7", "CL8", "CL9", "CL10", "CL6_UP", "CL7_UP", "CL8_UP", "CL9_UP", "CL10_UP"]:
            self.BM.moveRotate(-60) # 旋转到-60度
            self.BM.moveForward(-0.4) # 后退0.4米
        self.navigate_posekey(posekey) # 再次导航到货架点位

class MainController(MainController): # 主控制器类
    def run(self):
        # INIT
        self.CURRENT_STATE = "INIT"
        self.CURRENT_LOCATION = "INIT"
        self.welcome() # 欢迎界面 按下ENTER开始跑车
        self.calibratePose("start_left") # 校准起始位姿

        # PHOTO
        self.CURRENT_STATE = "PHOTO"
        self.CURRENT_LOCATION = "start_left"
        if not self.resume_mode: # 救援模式不需要拍照
            self.mail_box.append({'box_id': "U", 'result': 0, 'side': 'U'}) # 无效箱子
            self.takeboxPic_LD() # 邮箱拍照[左下]
            self.takeshelfPic_L() # 货架拍照[左侧]
            self.takeboxPic_LU() # 邮箱拍照[左上]
            self.takeboxPic_RU() # 邮箱拍照[右上]
            self.takeshelfPic_R() # 货架拍照[右侧]
            self.takeboxPic_RD() # 邮箱拍照[右下]
            self.handle_failed_boxes() # 处理失败的邮箱
        else:
            print("救援模式已启用，跳过拍照环节！")
            print("静态mail_box已获取！")
            print("静态mail_table已获取！")
            print("动态platform_state已获取！")
            print("动态completed_ids已获取！")
            print("动态grabbed_mails已获取！")
        self.navigate_posekey("start")

        # RUN
        self.CURRENT_STATE = "RUN"
        self.CURRENT_LOCATION = "start"
        mail_table_RSRB = self.select_side('R') # 右右列表
        mail_table_LSLB = self.select_side('L') # 左左列表
        mail_table_RSLB = self.select_non_side('R') # 右架左箱列表
        mail_table_LSRB = self.select_non_side('L') # 左架右箱列表

        if self.resume_mode and self.platform_state != 0:
            print("救援模式小车平台存在剩余邮件待处理！")
            self.resume_process_mails() # 恢复处理未完成的邮件
        
        if self.resume_mode:
            # 删除已完成的邮件并重排
            mail_table_RSRB = self.delete_completed_mails(mail_table_RSRB)
            mail_table_LSLB = self.delete_completed_mails(mail_table_LSLB)
            mail_table_RSLB = self.delete_completed_mails(mail_table_RSLB)
            mail_table_LSRB = self.delete_completed_mails(mail_table_LSRB)

        print("开始处理右区邮件...")
        self.process_mails(mail_table_RSRB)

        print("开始处理左区邮件...")
        self.process_mails(mail_table_LSLB)

        print("开始处理跨区邮件...")
        self.process_non_side_mails(shelf_R=mail_table_RSLB, shelf_L=mail_table_LSRB)

        # END
        self.state_manager.clear_state() # 清除之前保存的状态
        self.end() # 结束界面

if __name__ == "__main__":
    position_path = "/home/eaibot/nju_ws/src/motion_control/config/position.txt"
    navigate_path = "/home/eaibot/nju_ws/src/motion_control/config/nav_rules.json"
    state_save_path = "/home/eaibot/nju_ws/src/motion_control/config/state_save.pkl"
    log_path = "/home/eaibot/nju_ws/src/motion_control/config/state_log.txt"
    controller = MainController(position_path, navigate_path, state_save_path, log_path) # 创建主控制器实例
    controller.run() # 启动主控制器