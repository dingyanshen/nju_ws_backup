#! /usr/bin/env python2.7
# -*- coding:utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from motion_control.srv import *

class Lidar:
    def __init__(self):
        self.rangeFront = [425, 475]
        self.rangeRight = [805, 855]
        self.rangeLeft = [45, 95]
        self.rangeRightFront = [615, 665]
        self.rangeLeftFront = [235, 285]
        rospy.Subscriber("/scan", LaserScan, self._scan_CB)
        rospy.wait_for_message("/scan", LaserScan)
        rospy.Service("getFrontDist", getFrontDist, self._getFrontDist_CB)
        rospy.Service("getRightDist", getRightDist, self._getRightDist_CB)
        rospy.Service("getLeftDist", getLeftDist, self._getLeftDist_CB)
        rospy.Service(
            "getRightFrontDist", getRightFrontDist, self._getRightFrontDist_CB
        )
        rospy.Service("getLeftFrontDist", getLeftFrontDist, self._getLeftFrontDist_CB)
        rospy.Service("getDiffAngle", getDiffAngle, self._getDiffAngle_CB)
        self.pub_scanFiltered = rospy.Publisher(
            "/scanFiltered", LaserScan, queue_size=10
        )

    def _scan_CB(self, scan):
        self.scanData = scan
        self.scanRanges = self.scanData.ranges

    def _getFrontDist_CB(self, req):
        resp = getFrontDistResponse(self.getFrontDist())
        return resp

    def _getRightDist_CB(self, req):
        resp = getRightDistResponse(self.getRightDist())
        return resp

    def _getLeftDist_CB(self, req):
        resp = getLeftDistResponse(self.getLeftDist())
        return resp

    def _getRightFrontDist_CB(self, req):
        resp = getRightFrontDistResponse(self.getRightFrontDist())
        return resp

    def _getLeftFrontDist_CB(self, req):
        resp = getLeftFrontDistResponse(self.getLeftFrontDist())
        return resp

    def _getDiffAngle_CB(self, req):
        resp = getDiffAngleResponse(self.getDiffAngle())
        return resp

    def _getAverageDist(self, indexRange):
        start = indexRange[0]
        end = indexRange[1]
        scanRanges = self.scanRanges[int(start) : int(end)]
        dist = False
        while dist == False and not rospy.is_shutdown():
            count_zero = 0
            for i in range(len(scanRanges)):
                if scanRanges[i] == 0:
                    count_zero += 1
            if len(scanRanges) == count_zero:
                continue
            else:
                dist = sum(scanRanges) / (len(scanRanges) - count_zero)
        return dist

    def getFrontDist(self):
        return self._getAverageDist(self.rangeFront)

    def getRightDist(self):
        return self._getAverageDist(self.rangeRight)

    def getLeftDist(self):
        return self._getAverageDist(self.rangeLeft)

    def getRightFrontDist(self):
        return self._getAverageDist(self.rangeRightFront)

    def getLeftFrontDist(self):
        return self._getAverageDist(self.rangeLeftFront)

    def _leastSquares(self, x, y):
        N = len(x)
        sum_x = sum(x)
        sum_y = sum(y)
        sum_x2 = sum(x**2)
        sum_xy = sum(x * y)
        A = np.mat([[N, sum_x], [sum_x, sum_x2]])
        b = np.array([sum_y, sum_xy])
        return np.linalg.solve(A, b)

    def getDiffAngle(self):
        scanRanges = self.scanRanges[425:475]
        y = np.array(scanRanges) * 100
        x = np.arange(len(scanRanges))
        filter = y != 0
        x = x[filter]
        y = y[filter]
        b, k = self._leastSquares(x, y)
        angle = np.degrees(np.arctan(k))
        return angle

    def pubScanFiltered(self, indexRange):
        scanFiltered = self.scanData
        ranges = list(scanFiltered.ranges)
        start = indexRange[0]
        end = indexRange[1]
        for i in range(len(ranges)):
            ranges[i] = ranges[i] if i in range(start, end) else 0
        scanFiltered.ranges = tuple(ranges)
        self.pub_scanFiltered.publish(scanFiltered)

if __name__ == "__main__":
    rospy.init_node("lidar")
    LD = Lidar()
    rospy.spin()