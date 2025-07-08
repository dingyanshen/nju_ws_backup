#! /usr/bin/env python2.7
# -*- coding:utf-8 -*-
import time
import rospy
from camera.srv import PhotoService, PhotoServiceResponse, PhotoServiceRequest
from camera.srv import PhotoshelfService, PhotoshelfServiceResponse, PhotoshelfServiceRequest
from camera.srv import PhotoboxService, PhotoboxServiceResponse, PhotoboxServiceRequest
import cv2
import zmq

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://192.168.31.200:5555")

class PhotoServiceNode:
    def __init__(self):
        rospy.init_node('photo_service')
        self.service = rospy.Service('photo_service', PhotoService, self.camera_catch)
        self.service_shelf = rospy.Service('photo_shelf_service', PhotoshelfService, self.handle_shelf_photo)
        self.service_box = rospy.Service('photo_box_service', PhotoboxService, self.handle_box_photo)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            rospy.logerr("Camera initialization failed!")
            rospy.signal_shutdown("Camera initialization failed!")

    def camera_catch(self, req):
        # type 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20
        mode = req.type
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        for _ in range(10):
            self.cap.grab()
        ret, frame = self.cap.read()
        if not ret:
            rospy.logwarn("read camera failed")
            return PhotoServiceResponse(0, 0)
        image_name = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
        image_path = "/home/eaibot/nju_ws/src/camera/img/{}_{}_catch.jpg".format(image_name, mode)
        cv2.imwrite(image_path, frame, [cv2.IMWRITE_PNG_COMPRESSION, 0])
        socket.send_string(image_path)
        response = socket.recv()

        if response == b"0": # 硬切割
            if mode <= 10: # UP
                frame[0:480, 0:140] = (255, 255, 255)
                frame[0:480, 500:640] = (255, 255, 255)
                frame[240:480, 0:640] = (255, 255, 255)
            elif mode > 10: # DOWN
                frame[0:480, 0:140] = (255, 255, 255)
                frame[0:480, 500:640] = (255, 255, 255)
                frame[0:240, 0:640] = (255, 255, 255)
            image_name = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
            path = "/home/eaibot/nju_ws/src/camera/img_catch/{}_c.jpg".format(image_name)
            cv2.imwrite(path, frame, [cv2.IMWRITE_PNG_COMPRESSION, 0])
            socket.send_string(path)
            response = socket.recv()
            
        with open(response, 'r') as f:
            data = f.read().strip().split()
        if len(data) != 2:
            rospy.logwarn("Error reading error.txt")
            return PhotoServiceResponse(0, 0)
        try:
            error_x = float(data[0])
            error_y = float(data[1])
        except ValueError:
            rospy.logwarn("Error parsing error.txt")
            return PhotoServiceResponse(0, 0)
        return PhotoServiceResponse(error_x, error_y)
        
    def handle_shelf_photo(self, req):
        # type 1 2 3 4 5 6
        # int32[] results 0 1 2 3 4 5 6 7 8
        # int32[] positions_z #1 2
        # int32[] positions_x #1 2 3 4 5 6 7 8 9 10
        mode = req.type
        if req.type == 1 or req.type == 2 or req.type == 3:
            left_top = req.type * 2 - 1
        elif req.type == 4 or req.type == 5 or req.type == 6:
            left_top = req.type * 2 - 3
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
        for _ in range(10):
            self.cap.grab()
        ret, frame = self.cap.read()
        if not ret:
            rospy.logwarn("read camera failed")
            return PhotoshelfServiceResponse([], [], [])
        image_name = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
        image_path_sdo = "/home/eaibot/nju_ws/src/camera/img/{}_{}_sdo.jpg".format(image_name, mode)
        cv2.imwrite(image_path_sdo, frame, [cv2.IMWRITE_PNG_COMPRESSION, 0])
        socket.send_string(image_path_sdo)
        response_sdo = socket.recv()

        if response_sdo == b"0":
            frame1 = frame[0:480, 0:640] # Left Top
            frame2 = frame[0:480, 640:1280] # Right Top
            frame3 = frame[480:960, 0:640] # Left Bottom
            frame4 = frame[480:960, 640:1280] # Right Bottom
            image_name = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
            path = "/home/eaibot/nju_ws/src/camera/img_shelf/{}_s".format(image_name)
            pathul = path + "_ul_1.jpg"
            pathur = path + "_ur_1.jpg"
            pathdl = path + "_dl_1.jpg"
            pathdr = path + "_dr_1.jpg"
            pathul_2 = path + "_ul_2.jpg"
            pathur_2 = path + "_ur_2.jpg"
            pathdl_2 = path + "_dl_2.jpg"
            pathdr_2 = path + "_dr_2.jpg"
            cv2.imwrite(pathul, frame1, [cv2.IMWRITE_PNG_COMPRESSION, 0])
            cv2.imwrite(pathur, frame2, [cv2.IMWRITE_PNG_COMPRESSION, 0])
            cv2.imwrite(pathdl, frame3, [cv2.IMWRITE_PNG_COMPRESSION, 0])
            cv2.imwrite(pathdr, frame4, [cv2.IMWRITE_PNG_COMPRESSION, 0])
            cv2.imwrite(pathul_2, frame1, [cv2.IMWRITE_PNG_COMPRESSION, 0])
            cv2.imwrite(pathur_2, frame2, [cv2.IMWRITE_PNG_COMPRESSION, 0])
            cv2.imwrite(pathdl_2, frame3, [cv2.IMWRITE_PNG_COMPRESSION, 0])
            cv2.imwrite(pathdr_2, frame4, [cv2.IMWRITE_PNG_COMPRESSION, 0])
        else:
            path = response_sdo
            pathul = path + "_ul_1.jpg"
            pathur = path + "_ur_1.jpg"
            pathdl = path + "_dl_1.jpg"
            pathdr = path + "_dr_1.jpg"
            pathul_2 = path + "_ul_2.jpg"
            pathur_2 = path + "_ur_2.jpg"
            pathdl_2 = path + "_dl_2.jpg"
            pathdr_2 = path + "_dr_2.jpg"
        
        socket.send_string(pathul)
        response_qrcode_barcodes_1 = socket.recv()
        socket.send_string(pathur)
        response_qrcode_barcodes_2 = socket.recv()
        socket.send_string(pathdl)
        response_qrcode_barcodes_3 = socket.recv()
        socket.send_string(pathdr)
        response_qrcode_barcodes_4 = socket.recv()

        socket.send_string(pathul_2)
        response_qrcode_points_1 = socket.recv()
        socket.send_string(response_qrcode_points_1)
        response_1 = socket.recv()
        socket.send_string(pathur_2)
        response_qrcode_points_2 = socket.recv()
        socket.send_string(response_qrcode_points_2)
        response_2 = socket.recv()
        socket.send_string(pathdl_2)
        response_qrcode_points_3 = socket.recv()
        socket.send_string(response_qrcode_points_3)
        response_3 = socket.recv()
        socket.send_string(pathdr_2)
        response_qrcode_points_4 = socket.recv()
        socket.send_string(response_qrcode_points_4)
        response_4 = socket.recv()

        if(response_1 != response_qrcode_barcodes_1):
            if response_1 == 0 and response_qrcode_barcodes_1 != 0:
                response_1 = response_qrcode_barcodes_1
            elif response_1 != 0 and response_qrcode_barcodes_1 == 0:
                response_1 = response_1
            else:
                response_1 = 0
        if(response_2 != response_qrcode_barcodes_2):
            if response_2 == 0 and response_qrcode_barcodes_2 != 0:
                response_2 = response_qrcode_barcodes_2
            elif response_2 != 0 and response_qrcode_barcodes_2 == 0:
                response_2 = response_2
            else:
                response_2 = 0
        if(response_3 != response_qrcode_barcodes_3):
            if response_3 == 0 and response_qrcode_barcodes_3 != 0:
                response_3 = response_qrcode_barcodes_3
            elif response_3 != 0 and response_qrcode_barcodes_3 == 0:
                response_3 = response_3
            else:
                response_3 = 0
        if(response_4 != response_qrcode_barcodes_4):
            if response_4 == 0 and response_qrcode_barcodes_4 != 0:
                response_4 = response_qrcode_barcodes_4
            elif response_4 != 0 and response_qrcode_barcodes_4 == 0:
                response_4 = response_4
            else:
                response_4 = 0

        num_province = ['无效', '江苏', '浙江', '安徽', '河南', '湖南', '四川', '广东', '福建']
        print(num_province[int(response_1)])
        print(num_province[int(response_2)])
        print(num_province[int(response_3)])
        print(num_province[int(response_4)])
        return PhotoshelfServiceResponse([int(response_1), int(response_2), int(response_3), int(response_4)], [1, 1, 2, 2], [left_top, left_top + 1, left_top, left_top + 1])
    
    def handle_box_photo(self, req):
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        for _ in range(10):
            self.cap.grab()
        ret, frame = self.cap.read()
        frame = frame[160:240, 0:320]
        if not ret:
            rospy.logwarn("read camera failed")
            return PhotoboxServiceResponse(0)
        image_name = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
        image_path = "/home/eaibot/nju_ws/src/camera/img/{}_box.jpg".format(image_name)
        cv2.imwrite(image_path, frame, [cv2.IMWRITE_PNG_COMPRESSION, 0]) 
        socket.send_string(image_path)
        response = socket.recv()
        num_province = ['无效', '江苏', '浙江', '安徽', '河南', '湖南', '四川', '广东', '福建']
        print(num_province[int(response)])
        return PhotoboxServiceResponse(int(response))
    
if __name__ == "__main__":
    node = PhotoServiceNode()
    rospy.spin()