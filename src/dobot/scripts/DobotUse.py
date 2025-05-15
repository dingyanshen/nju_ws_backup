#! /usr/bin/env python2.7
# -*- coding:utf-8 -*-

import rospy
import Action
from dobot.srv import GraspService, GraspServiceResponse
from dobot.srv import ThrowService, ThrowServiceResponse

class DobotServiceNode:
    def __init__(self):
        rospy.init_node('dobot_service')
        self.service_grasp = rospy.Service('dobot_grasp_service', GraspService, self.handle_grasp)
        self.service_throw = rospy.Service('dobot_throw_service', ThrowService, self.handle_throw)
        self.dobot = Action.Dobot()
        self.dobot.setPTPCommonParams(1000.0, 1000.0)
        self.dobot.setPTPCoordinateParams(1000, 1000, 200, 200)
        # self.dobot.setHome()
        self.dobot.clearAlarmsState()
        self.dobot.setIOMultiplexing(8, 2)
        self.dobot.setPose(250,0,0,0)
        rospy.sleep(0.1)
        self.dobot.setTheta(0)

    def handle_grasp(self, req):
        rospy.loginfo("Grasping mail...")
        try:
            self.dobot.CatchBox(req.shelf_z, req.pos_z, req.error_x, req.error_y)
            return GraspServiceResponse(True)
        except Exception as e:
            rospy.logerr("Grasping failed: {e}")
            return GraspServiceResponse(False)
        
    def handle_throw(self, req):
        rospy.loginfo("Throwing mail...")
        try:
            self.dobot.ThrowBox(req.pos_z, req.mailbox_pos)
            return ThrowServiceResponse(True)
        except Exception as e:
            rospy.logerr("Throwing failed: {e}")
            return ThrowServiceResponse(False)

if __name__ == "__main__":
    node = DobotServiceNode()
    rospy.spin()