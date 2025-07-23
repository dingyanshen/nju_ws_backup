#! /usr/bin/env python2.7
# -*- coding:utf-8 -*-
import time
import rospy
from camera.srv import PhotoService, PhotoServiceResponse, PhotoServiceRequest
from camera.srv import PhotoshelfService, PhotoshelfServiceResponse, PhotoshelfServiceRequest
from camera.srv import PhotoboxService, PhotoboxServiceResponse, PhotoboxServiceRequest
import cv2
import zmq
from take_photo import PhotoServiceNode

# 初始化zmq 与server.py建立连接
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://192.168.31.200:5555")

if __name__ == "__main__":
    photo = PhotoServiceNode()
    while True:
        num = raw_input("请输入邮件编号(1-20): ")
        if num.isdigit() and 1 <= int(num) <= 20:
            req = PhotoServiceRequest()
            req.type = int(num)
            response = photo.camera_catch(req)
            print(response)
        else:
            print("输入无效，请输入1-20之间的数字！")