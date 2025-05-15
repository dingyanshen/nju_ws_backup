# -*- coding:utf-8 -*-
import cv2
import zmq
import easyocr
import time
import numpy as np
import pnp

def calculate(frame, points):
    points = points[0]
    min_x = np.min(points[:, 0])
    max_x = np.max(points[:, 0])
    min_y = np.min(points[:, 1])
    max_y = np.max(points[:, 1])
    side_length = max(max_x - min_x, max_y - min_y)
    square_min_x = min_x
    square_min_y = min_y
    square_max_x = min_x + side_length
    square_max_y = min_y + side_length
    adjusted_coordinates = {
        "top_left": (square_min_x - 3.0 * side_length, square_min_y + 0.4 * side_length),
        "bottom_left": (square_min_x - 3.0 * side_length, square_max_y),
        "top_right": (square_max_x - 0.8 * side_length, square_min_y + 0.4 * side_length),
        "bottom_right": (square_max_x - 0.8 * side_length, square_max_y)
    }
    crop_min_x = int(adjusted_coordinates["top_left"][0])
    crop_min_y = int(adjusted_coordinates["top_left"][1])
    crop_max_x = int(adjusted_coordinates["bottom_right"][0])
    crop_max_y = int(adjusted_coordinates["bottom_right"][1])
    crop_min_x = max(0, crop_min_x)
    crop_min_y = max(0, crop_min_y)
    crop_max_x = min(frame.shape[1], crop_max_x)
    crop_max_y = min(frame.shape[0], crop_max_y)
    cropped_image = frame[crop_min_y:crop_max_y, crop_min_x:crop_max_x]
    image_name = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
    image_cropped_path = "/home/eaibot/nju_ws/src/camera/img/{}_cropped.jpg".format(image_name)
    cv2.imwrite(image_cropped_path, cropped_image, [cv2.IMWRITE_PNG_COMPRESSION, 0])
    return image_cropped_path

def qrcode_barcodes():
    img = cv2.imread(message)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    barcodes, points = model.detectAndDecode(gray)
    text_num = find_province_number(barcodes, province_match, province_num)
    socket.send(str(text_num).encode())

def qrcode_points():
    img = cv2.imread(message)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    barcodes, points = model.detectAndDecode(gray)
    path = calculate(img, points)
    if points is not None:
        socket.send(str(path).encode())
    else:
        socket.send(b"0")

def shelf():
    img = cv2.imread(message)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    results = reader.readtext(gray)
    text_results = [result[1] for result in results]
    print(text_results)
    text_num = find_province_number_double(text_results, province_match_double)
    socket.send(str(text_num).encode())

def box():
    img = cv2.imread(message)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    results = reader.readtext(gray)
    text_results = [result[1] for result in results]
    print(text_results)
    text_num = find_province_number(text_results, province_match, province_num)
    socket.send(str(text_num).encode())

def catch():
    img = cv2.imread(message)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    barcodes, points = model.detectAndDecode(gray)
    if points is None:
        error_x = error_y = 0
    else:
        error_x, error_y = pnp(points)
    path = "/home/eaibot/nju_ws/src/camera/config/error.txt"
    with open(path, 'w') as f:
        f.write(f"{error_x} {error_y}")
    socket.send(str(path).encode())

def find_province_number(results, match, num):
    for item in results:
        for char in item:
            if char in match:
                index = match.index(char)
                return num[index]
    return 0

def find_province_number_double(results, match_double):
    for item in results:
        for province in match_double:
            if province in item:
                return match_double.index(province) + 1
    return 0

if __name__ == "__main__":
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5555")
    depro = '/home/eaibot/nju_ws/src/camera/config/detect.prototxt'
    decaf = '/home/eaibot/nju_ws/src/camera/config/detect.caffemodel'
    srpro = '/home/eaibot/nju_ws/src/camera/config/sr.prototxt'
    srcaf = '/home/eaibot/nju_ws/src/camera/config/sr.caffemodel'
    model = cv2.wechat_qrcode_WeChatQRCode(depro, decaf, srpro, srcaf)
    reader = easyocr.Reader(['ch_sim', 'en'])
    province_match = ['苏', '浙', '安', '徽', '河', '湖', '四', '川', '广', '东', '福', '建']
    province_match_double =['江苏', '浙江', '安徽', '河南', '湖南', '四川', '广东', '福建']
    province_num = [1, 2, 3, 3, 4, 5, 6, 6, 7, 7, 8, 8]
    print("Server is running...")
    while True:
        message = socket.recv_string()
        try:
            if message.endswith("qrcode_barcodes_1.jpg") or message.endswith("qrcode_barcodes_2.jpg") or message.endswith("qrcode_barcodes_3.jpg") or message.endswith("qrcode_barcodes_4.jpg"):
                qrcode_barcodes()
            elif message.endswith("qrcode_points_1.jpg") or message.endswith("qrcode_points_2.jpg") or message.endswith("qrcode_points_3.jpg") or message.endswith("qrcode_points_4.jpg"):
                qrcode_points()
            elif message.endswith("box.jpg"):
                box()
            elif message.endswith("cropped.jpg"):
                shelf()
            elif message.endswith("catch.jpg"):
                catch()
            else:
                print("Invalid message received:", message)
                socket.send(b"0")
        except Exception as e:
            socket.send(b"0")