# -*- coding:utf-8 -*-
print("初始化摄像头服务端...")
import cv2
import zmq
import easyocr
import time
import numpy as np
import pnp
from pupil_apriltags import Detector

def detect_apriltags(image_path, target_tags=[1, 11]):
    # 货架拍照：检测图片特定AprilTags 返回-1或四张切割图片路径

    image_name = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
    path = "/home/eaibot/nju_ws/src/camera/img_shelf/{}_s.jpg".format(image_name)

    image = cv2.imread(image_path)
    if image is None:
        raise ValueError(f"无法读取图片: {image_path}")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    detections = detector.detect(gray)
    tag_positions_x={}
    tag_positions_y={}

    for det in detections:
        tag_id = det.tag_id
        if tag_id in target_tags:
            center =  det.center.astype(int)
            x, y = center
            tag_positions_x[tag_id] = x
            tag_positions_y[tag_id] = y

    if len(tag_positions_x) < 2: # 检测到属于目标标签的数量小于2
        return -1
    
    x=(tag_positions_x[target_tags[0]]+tag_positions_x[target_tags[1]])//2
    y=(3*tag_positions_y[target_tags[0]]+tag_positions_y[target_tags[1]])//4
    H=(tag_positions_y[target_tags[1]]-tag_positions_y[target_tags[0]])
    W=int(H*15.5/11.5)

    h,w=image.shape[:2]
    l=max(0,x-W)
    r=min(w,x+W)
    u=max(0,y-H)
    d=min(h,y+H)

    pic_ul=image[u:y,l:x]
    pic_ur=image[u:y,x:r]
    pic_dl=image[y:d,l:x]
    pic_dr=image[y:d,x:r]
    
    path1 = path + "_dl_1.jpg"
    cv2.imwrite(path1,pic_dl)
    path2 = path + "_dr_1.jpg"
    cv2.imwrite(path2,pic_dr)
    path3 = path + "_ul_1.jpg"
    cv2.imwrite(path3,pic_ul)
    path4 = path + "_ur_1.jpg"
    cv2.imwrite(path4,pic_ur)

    path1_2 = path + "_dl_2.jpg"
    cv2.imwrite(path1_2,pic_dl)
    path2_2 = path + "_dr_2.jpg"
    cv2.imwrite(path2_2,pic_dr)
    path3_2 = path + "_ul_2.jpg"
    cv2.imwrite(path3_2,pic_ul)
    path4_2 = path + "_ur_2.jpg"
    cv2.imwrite(path4_2,pic_ur)

    return path

def detect_apriltags_catch(image_path, target_tags=[1, 11], height=1, location=1):
    # 抓取时拍照：检测图片特定AprilTags 返回-1或填充后的图片路径

    image_name = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime())
    path = "/home/eaibot/nju_ws/src/camera/img_catch/{}_c.jpg".format(image_name)

    # 最后一列特殊处理标志
    LAST_COLUMN_TAGS = False
    if target_tags[0] == 10 and target_tags[1] == 20:
        LAST_COLUMN_TAGS = True

    # 右侧非最后一列偏移
    if location == 2 and LAST_COLUMN_TAGS == False:
        target_tags = [target_tags[0] + 1, target_tags[1] + 1]
    
    # [1,11]:1,11 [2,12]:2,12 [3,13]:3,13 [4,14]:4,14 [5,15]:5,15
    # [7,17]:6,16 [8,18]:7,17 [9,19]:8,18 [10,20]:9,19 [10,20]:10,20
    
    image = cv2.imread(image_path)
    if image is None:
        raise ValueError(f"无法读取图片: {image_path}")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    detections = detector.detect(gray)
    tag_positions_x={}
    tag_positions_y={}

    for det in detections:
        tag_id = det.tag_id
        if tag_id in target_tags:
            center =  det.center.astype(int)
            x, y = center
            tag_positions_x[tag_id] = x
            tag_positions_y[tag_id] = y

    if len(tag_positions_x) < 2: # 检测到属于目标标签的数量小于2
        if LAST_COLUMN_TAGS == True: # 如果是因为最后一列
            if height == 1:
                image[0:480, 0:140] = (255, 255, 255)
                image[240:480, 0:640] = (255, 255, 255)
            elif height == 2:
                image[0:480, 0:140] = (255, 255, 255)
                image[0:240, 0:640] = (255, 255, 255)
            cv2.imwrite(path,image)
            return path
        return -1
            
    x=(tag_positions_x[target_tags[0]]+tag_positions_x[target_tags[1]])//2
    y=(3*tag_positions_y[target_tags[0]]+tag_positions_y[target_tags[1]])//4
    H=(tag_positions_y[target_tags[1]]-tag_positions_y[target_tags[0]])
    W=int(H*15.5/11.5)
    bg_color = (255, 255, 255)

    h,w=image.shape[:2]
    l=max(0,x-W)
    r=min(w,x+W)
    u=max(0,y-H)
    d=min(h,y+H)

    pic_ul=np.full((h, w, 3), bg_color, dtype=np.uint8)
    pic_ul[u:y,l:x]=image[u:y,l:x]

    pic_ur=np.full((h, w, 3), bg_color, dtype=np.uint8)
    pic_ur[u:y,x:r]=image[u:y,x:r]

    pic_dl=np.full((h, w, 3), bg_color, dtype=np.uint8)
    pic_dl[y:d,l:x]=image[y:d,l:x]

    pic_dr=np.full((h, w, 3), bg_color, dtype=np.uint8)
    pic_dr[y:d,x:r]=image[y:d,x:r]

    if LAST_COLUMN_TAGS == True:
        if height == 1:
            cv2.imwrite(path,pic_ur)
        elif height == 2:
            cv2.imwrite(path,pic_dr)
    else:
        if height == 1:
            cv2.imwrite(path,pic_ul)
        elif height == 2:
            cv2.imwrite(path,pic_dl)
    return path

def calculate(frame, points):
    # 根据图片及二维码角点坐标预测地址位置并切割 返回切割图片地址
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
    # [1]接收切割后的四个图片路径，返回二维码识别结果
    img = cv2.imread(message)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    barcodes, points = model.detectAndDecode(gray)
    text_num = find_province_number(barcodes, province_match, province_num)
    socket.send(str(text_num).encode())

def qrcode_points():
    # [2]接收切割后的四个图片路径，返回cropped图片路径
    img = cv2.imread(message)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    barcodes, points = model.detectAndDecode(gray)
    path = calculate(img, points)
    if points is not None:
        socket.send(str(path).encode())
    else:
        socket.send(b"0")

def shelf():
    # [cropped]接收cropped路径，返回文字识别结果
    img = cv2.imread(message)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    results = reader.readtext(gray)
    text_results = [result[1] for result in results]
    print("识别结果:", text_results)
    text_num = find_province_number_double(text_results, province_match_double, province_num_double)
    socket.send(str(text_num).encode())

def box():
    #[box]接收快递箱拍照的图片路径，返回文字识别结果
    img = cv2.imread(message)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # 预留参数
    # detail=1, paragraph=False, text_threshold=0.35,
    # low_text=0.15, link_threshold=0.1, contrast_ths=0.05,
    # adjust_contrast=0.9, mag_ratio=1.0, add_margin=0.3,
    # slope_ths=0.05, ycenter_ths=0.7, height_ths=0.7, width_ths=0.9
    results = reader.readtext(gray)
    text_results = [result[1] for result in results]
    print("识别结果:", text_results)
    text_num = find_province_number(text_results, province_match, province_num)
    socket.send(str(text_num).encode())

def catch():
    # [catch]接收抓取时拍照的图片路径，返回切割失败或error路径
    # mode 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20
    mode = int(message.split('_')[-2])
    if mode == 1 or mode == 2 or mode == 3 or mode == 4 or mode == 5:
        height = 1 # UP
        location = 1 # LEFT
    elif mode == 6 or mode == 7 or mode == 8 or mode == 9 or mode == 10:
        height = 1 # UP
        location = 2 # RIGHT
    elif mode == 11 or mode == 12 or mode == 13 or mode == 14 or mode == 15:
        height = 2 # DOWN
        location = 1 # LEFT
    elif mode == 16 or mode == 17 or mode == 18 or mode == 19 or mode == 20:
        height = 2 # DOWN
        location = 2 # RIGHT
    if mode > 10:
        mode1 = mode - 10
        mode2 = mode
    elif mode <= 10:
        mode1 = mode
        mode2 = mode + 10
    path = detect_apriltags_catch(message, target_tags=[mode1, mode2], height=height, location=location)
    if path == -1:  # 检测到属于目标标签的数量小于2
        socket.send(b"0")
    else:
        img = cv2.imread(path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        barcodes, points = model.detectAndDecode(gray)
        if len(points) > 1:
            error_x = error_y = [0]
        if len(points) == 0:
            error_x = error_y = [0]
        else:
            error_x, error_y = pnp.pnp(points)
        path = "/home/eaibot/nju_ws/src/camera/config/error.txt"
        with open(path, 'w') as f:
            f.write(f"{error_x[0]} {error_y[0]}")
        socket.send(str(path).encode())

def sdo():
    # [sdo]接收货架拍照的图片路径，返回切割失败或切割成功的路径
    mode = int(message.split('_')[-2])
    if mode == 1:
        _mode = [1, 11]
    elif mode == 2:
        _mode = [3, 13]
    elif mode == 3:
        _mode = [5, 15]
    elif mode == 4:
        _mode = [6, 16]
    elif mode == 5:
        _mode = [8, 18]
    elif mode == 6:
        _mode = [10, 20]
    path = detect_apriltags(message, target_tags=_mode)
    if path == -1:
        # 检测到属于目标标签的数量小于2
        socket.send(b"0")
    else:
        socket.send(str(path).encode())

def c_hard():
    # [c]接收硬切割后的图片路径，返回error路径
    img = cv2.imread(message)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    barcodes, points = model.detectAndDecode(gray)
    if len(points) > 1:
        error_x = error_y = [0]
    if len(points) == 0:
        error_x = error_y = [0]
    else:
        error_x, error_y = pnp.pnp(points)
    path = "/home/eaibot/nju_ws/src/camera/config/error.txt"
    with open(path, 'w') as f:
        f.write(f"{error_x[0]} {error_y[0]}")
    socket.send(str(path).encode())

def find_province_number(results, match, num):
    # 邮箱OCR识别单字匹配
    for item in results:
        for char in item:
            if char in match:
                index = match.index(char)
                return num[index]
    return 0

def find_province_number_double(results, match_double, num_double):
    # 货架OCR识别双字匹配
    for item in results:
        for province in match_double:
            if province in item:
                index = match_double.index(province)
                return num_double[index]
    return 0

if __name__ == "__main__":
    # 初始化zmq 与拍照节点建立连接
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5555")
    # 初始化检测模型
    depro = '/home/eaibot/nju_ws/src/camera/config/detect.prototxt'
    decaf = '/home/eaibot/nju_ws/src/camera/config/detect.caffemodel'
    srpro = '/home/eaibot/nju_ws/src/camera/config/sr.prototxt'
    srcaf = '/home/eaibot/nju_ws/src/camera/config/sr.caffemodel'
    detector = Detector(families='tag36h11', nthreads=1, quad_decimate=1.0, quad_sigma=0.0, refine_edges=1, decode_sharpening=0.25, debug=0)
    model = cv2.wechat_qrcode_WeChatQRCode(depro, decaf, srpro, srcaf)
    reader = easyocr.Reader(['ch_sim', 'en'])

    # 初始化省份兼容性匹配列表

    province_match = [# '江', '讧',
                        '苏', '茆', '芴',
                      
                        '浙', '断', '祈',
                      # '江', '讧',

                        '安', '女', '妄', '桉',
                        '徽', '徵', '傲', '僦', '蔹',

                        '河',
                      # '南', '雨',

                        '湖',
                      # '南',

                        '四', '凹',
                        '川',
                      
                        '广',
                        '东', '玄', '套',

                        '福', '梅', '橱', '橘',
                        '建', '处', '廷', '迹',]
    
    province_num = [  # 1, 1,
                        1, 1, 1,
                    
                        2, 2, 2,
                      # 2, 2,

                        3, 3, 3, 3,
                        3, 3, 3, 3, 3,

                        4,
                      # 4, 4,

                        5,
                      # 5,

                        6, 6,
                        6,

                        7,
                        7, 7, 7,

                        8, 8, 8, 8,
                        8, 8, 8, 8,]
    
    province_match_double = ['江苏', '讧苏',
                             '江茆', '讧茆',
                             '江芴', '讧芴',

                             '浙江', '断江', '祈江',
                             '浙讧', '断讧', '祈讧',

                             '安徽', '女徽', '妄徽', '桉徽',
                             '安徵', '女徵', '妄徵', '桉徵',
                             '安傲', '女傲', '妄傲', '桉傲',
                             '安僦', '女僦', '妄僦', '桉僦',
                             '安蔹', '女蔹', '妄蔹', '桉蔹',

                             '河南',
                             '河雨',

                             '湖南',

                             '四川', '凹川',

                             '广东',
                             '广玄',
                             '广套',

                             '福建', '梅建', '橱建', '橘建',
                             '福处', '梅处', '橱处', '橘处',
                             '福廷', '梅廷', '橱廷', '橘廷',
                             '福迹', '梅迹', '橱迹', '橘迹',]
    
    province_num_double = [1, 1,
                           1, 1,
                           1, 1,

                           2, 2, 2,
                           2, 2, 2,

                           3, 3, 3, 3,
                           3, 3, 3, 3,
                           3, 3, 3, 3,
                           3, 3, 3, 3,
                           3, 3, 3, 3,

                           4,
                           4,

                           5,

                           6, 6,

                           7,
                           7,
                           7,

                           8, 8, 8, 8,
                           8, 8, 8, 8,
                           8, 8, 8, 8,
                           8, 8, 8, 8,]
    
    print("摄像头服务端已启动，等待处理图片...")
    while True:
        # 接收拍照节点发送的图片路径
        message = socket.recv_string()
        print("处理图片:" + message)
        try:
            if message.endswith("1.jpg"):
                qrcode_barcodes()
            elif message.endswith("2.jpg"):
                qrcode_points()
            elif message.endswith("box.jpg"):
                box()
            elif message.endswith("cropped.jpg"):
                shelf()
            elif message.endswith("catch.jpg"):
                catch()
            elif message.endswith("sdo.jpg"):
                sdo()
            elif message.endswith("c.jpg"):
                c_hard()
            else:
                print("图片路径存在问题：" + message)
                socket.send(b"0")
        except Exception as e:
            socket.send(b"0")