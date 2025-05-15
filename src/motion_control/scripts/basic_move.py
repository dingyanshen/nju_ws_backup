#! /usr/bin/env python2.7
# -*- coding:utf-8 -*-

import rospy
import PyKDL
from geometry_msgs.msg import Pose, Twist
from math import sqrt, pow, pi, radians
from pid import PID
from lidar import Lidar

class BasicMove:
    def __init__(self, detailInfo=False, recordData=False):
        self.LD = Lidar()
        self.detailInfo = detailInfo
        self.recordData = recordData
        self.file_path = "/home/eaibot/nju_ws/src/motion_control/data/"
        self.rateControl = rospy.Rate(500)
        self.PID_forward = PID(-3, -0.00, -0.00, setpoint=0.0)
        self.PID_forward.output_limits = (-0.3, 0.3)
        self.toleranceForward = 0.01
        self.PID_cross = PID(1, -0.0, -0.0, setpoint=0.0)
        self.PID_cross.output_limits = (-0.1, 0.1)
        self.PID_rotate = PID(-5, -0.0018, -0.0008, setpoint=0.0)
        self.PID_rotate.output_limits = (-0.5, 0.5)
        self.toleranceRotate = 0.01
        rospy.Subscriber("/robot_pose", Pose, self._robot_pose_CB, queue_size=10)
        rospy.wait_for_message("/robot_pose", Pose)
        self.pub_velCmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def _robot_pose_CB(self, pose):
        self.poseCur = pose

    def _writeDataToFile(self, times, errors, file_name):
        file = open(self.file_path + file_name, "w")
        for i in range(len(times)):
            data = times[i] + " " + errors[i] + "\n"
            file.write(data)
        file.close()

    def _getDistance(self, pose1, pose2):
        x1, x2 = pose1.position.x, pose2.position.x
        y1, y2 = pose1.position.y, pose2.position.y
        return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2))

    def _getForwardErrorF(self, poseInit, targetDist, mode="normal"):
        if mode == "lidar":
            distFront = self.LD.getFrontDist()
            error = distFront - targetDist
        else:
            distance = self._getDistance(poseInit, self.poseCur)
            if targetDist > 0:
                error = targetDist - distance
            else:
                error = targetDist + distance
        return error

    def _getForwardErrorC(self, mode="normal"):
        if mode == "lidar":
            return 0
        else:
            return 0

    def _pubVelCmd(self, linear_x=0.0, angular_z=0.0):
        velCmd = Twist()
        velCmd.linear.x = linear_x
        velCmd.angular.z = angular_z
        self.pub_velCmd.publish(velCmd)

    def moveForward(self, targetDist, mode="normal"): # 直线运动
        targetDist = float(targetDist)
        self.PID_forward.reset()
        self.PID_cross.reset()
        if self.recordData:
            timeStart = rospy.Time.now()
            times, errors = [], []
        if self.detailInfo == True:
            print("move forward " + str(targetDist) + "m with " + str(mode) + " mode")
        self.poseInit = self.poseCur
        error_f = self._getForwardErrorF(self.poseInit, targetDist, mode)
        error_c = self._getForwardErrorC(mode=mode)
        while abs(error_f) > self.toleranceForward and not rospy.is_shutdown():
            if self.recordData:
                time = rospy.Time.now() - timeStart
                times.append(str(time))
                errors.append(str(error_f))
            linear_x = self.PID_forward(error_f)
            angular_z = self.PID_cross(error_c) if mode == "lidar" else 0
            angular_z = angular_z if linear_x > 0 else -angular_z
            self._pubVelCmd(linear_x=linear_x, angular_z=angular_z)
            error_f = self._getForwardErrorF(self.poseInit, targetDist, mode)
            error_c = self._getForwardErrorC(mode=mode)
            self.rateControl.sleep()
        self.stop()
        if self.recordData:
            self._writeDataToFile(times, errors, "forward_" + str(targetDist) + ".txt")

    def _quatToRadians(self, quaternion_z, quaternion_w):
        return PyKDL.Rotation.Quaternion(0, 0, quaternion_z, quaternion_w).GetRPY()[2]

    def _getRadians(self, pose):
        return self._quatToRadians(pose.orientation.z, pose.orientation.w)

    def _getRotateError(self, radiansTarget, radiansCur, mode="normal"):
        if mode == "lidar":
            angleDiff = self.LD.getDiffAngle()
            angleDiff = radians(angleDiff)
        else:
            angleDiff = radiansTarget - radiansCur
            while angleDiff > pi:
                angleDiff -= 2 * pi
            while angleDiff < -pi:
                angleDiff += 2 * pi
        return angleDiff

    def moveRotate(self, targetDegree, mode="normal"): # 自转运动
        radiansTarget = radians(float(targetDegree))
        self.PID_rotate.reset()
        if self.recordData:
            timeStart = rospy.Time.now()
            times, errors = [], []
        if self.detailInfo == True:
            print(
                "move rotate to " + str(targetDegree) + "° with " + str(mode) + " mode"
            )
        radiansCur = self._getRadians(self.poseCur)
        error = self._getRotateError(radiansTarget, radiansCur, mode)
        while abs(error) > self.toleranceRotate and not rospy.is_shutdown():
            if self.recordData:
                time = rospy.Time.now() - timeStart
                times.append(str(time))
                errors.append(str(error))
            angular_z = self.PID_rotate(error)
            self._pubVelCmd(angular_z=angular_z)
            radiansCur = self._getRadians(self.poseCur)
            error = self._getRotateError(radiansTarget, radiansCur, mode)
            self.rateControl.sleep()
        self.stop()
        if self.recordData:
            self._writeDataToFile(times, errors, "rotate_" + str(targetDegree) + ".txt")

    def moveArc(self, R, degree, direction, linear_x=0.2): # 弧线运动
        R, degree, linear_x = float(R), float(degree), float(linear_x)
        if direction not in ("LEFT", "RIGHT"):
            print("direction must be LEFT or RIGHT")
            return
        if self.detailInfo == True:
            print(
                "move arc with radius: "
                + str(R)
                + "m, degree: "
                + str(degree)
                + "°, direction: "
                + str(direction)
                + ", linear_x: "
                + str(linear_x)
            )
        theta = radians(degree)
        time = theta * R / abs(linear_x)
        direction = 1 if direction == "LEFT" else -1
        angular_z = abs(linear_x) / R * direction
        timeInit = rospy.get_time()
        timeCur = timeInit
        while timeCur - timeInit < time and not rospy.is_shutdown():
            self._pubVelCmd(linear_x=linear_x, angular_z=angular_z)
            timeCur = rospy.get_time()
            rospy.sleep(0.01)
        self.stop()

    def stop(self): # 停止运动
        self.pub_velCmd.publish(Twist())

if __name__ == "__main__":
    rospy.init_node("basic_move")
    BM = BasicMove(recordData=True)
    rospy.spin()