#! /usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import time
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Int32, Header, Float32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from l22_aero_vision.msg import ColorRectMarker, ColorRectMarkerArray
# import tf



rospy.init_node('l22_aero_color_node', anonymous=True)
bridge = CvBridge()

markers_arr_pub = rospy.Publisher("/l22_aero_color/markers", ColorRectMarkerArray)
circles_arr_pub = rospy.Publisher("/l22_aero_color/circles", ColorRectMarkerArray)
image_pub = rospy.Publisher("/l22_aero_color/debug_img", Image)
'''
colors_p_hsv = {
    "yellow": (np.array([8,  60,  60]), np.array([35,  255, 255])),
    "red": (np.array([160, 80,  80]), np.array([255, 255, 255]), np.array([0, 80,  80]), np.array([8, 255, 255])),
    "blue": (np.array([161,  56,  109]), np.array([181, 126, 151])),
    "green": (np.array([90,  70,  70]), np.array([160, 255, 255])),
    "brown": (np.array([160, 80,  80]), np.array([255, 255, 255]), np.array([0, 80,  80]), np.array([8, 255, 255]))
}
'''


# Параметры цвета маркеров
# colors_p_hsv = {
#     'blue': (np.array([103, 47, 65]), np.array([150, 187, 172])),
#     'green': (np.array([28, 44, 20]), np.array([100, 255, 255])),
#     'yellow': (np.array([14, 100, 104]), np.array([29, 255, 255])),
#     'red': (np.array([151, 134, 99]), np.array([255, 243, 252])),
#     'brown': (np.array([6, 86, 99]), np.array([255, 243, 252]))
# }

# colors_p_hsv = {
#     'green': (np.array([71, 86, 22]), np.array([88, 255, 255])),
#     'yellow': (np.array([14, 75, 33]), np.array([37, 255, 255])),
#     'blue': (np.array([94, 95, 55]), np.array([132, 255, 255])),
#     'red': (np.array([168, 72, 61]), np.array([208, 255, 255])),
#     'brown': (np.array([0, 57, 34]), np.array([19, 253, 127]))
# }

colors_p_hsv = {
    'green': (np.array([71, 86, 22]), np.array([88, 255, 255])),
    'yellow': (np.array([14, 75, 33]), np.array([37, 255, 255])),
    'blue': (np.array([94, 88, 63]), np.array([134, 255, 255])),
    'red': (np.array([136, 78, 102]), np.array([174, 255, 255])),
    'brown': (np.array([0, 81, 91]), np.array([10, 255, 255])),
}

colors_p_rgb = {
    "yellow": [0,  200,  200],
    "red": [0, 0, 255],
    "blue": [255, 0, 0],
    "green": [0, 255, 0],
    "brown": [42, 42, 165]
}


# Формат вывода 
type_mapping = {
    'blue': 'N2_water',
    'green': 'N2_pastures',
    'yellow': 'N2_seed',
    'red': 'N2_potato',
    'brown': 'N2_soil'
}

# Размеры цветных маркеров 
MARKER_SIDE1_SIZE = 0.35 # in m
MARKER_SIDE2_SIZE = 0.35 # in m
OBJ_S_THRESH = 350
OFFSET = [61, 35]

CIRCLE_R = 0.2

objectPoint = np.array([(-MARKER_SIDE1_SIZE / 2, -MARKER_SIDE2_SIZE / 2, 0), (MARKER_SIDE1_SIZE / 2, -MARKER_SIDE2_SIZE / 2, 0), 
                        (MARKER_SIDE1_SIZE / 2, MARKER_SIDE2_SIZE / 2, 0), (-MARKER_SIDE1_SIZE / 2, MARKER_SIDE2_SIZE / 2, 0)])
objectPoint_circles = np.array([(-CIRCLE_R, -CIRCLE_R, 0), (CIRCLE_R, -CIRCLE_R, 0), 
                        (CIRCLE_R, CIRCLE_R, 0), (-CIRCLE_R, CIRCLE_R, 0)])
# print("objectPoint shape:", objectPoint.shape)
class ColorRect:
    def __init__(self, cx_img=0, cy_img=0, color="none", points_img=[]):
        self.cx_img = cx_img
        self.cy_img = cy_img
        self.color = color
        self.points_img = points_img
class ColorRectMarker_p:
    def __init__(self, cx_img=0, cy_img=0, color="none", points_img=[], cx_cam=0, cy_cam=0, cz_cam=0):
        self.cx_img = cx_img
        self.cy_img = cy_img
        self.color = color
        self.points_img = points_img
        self.cx_cam = cx_cam
        self.cy_cam = cy_cam
        self.cz_cam = cz_cam
    def fromColorRect(self, cr):
        self.cx_img = cr.cx_img
        self.cy_img = cr.cy_img
        self.color = cr.color
        self.points_img = cr.points_img
        return self
    def toMsg(self):
        return ColorRectMarker(color=self.color, cx_img=self.cx_img, cy_img=self.cy_img, cx_cam=self.cx_cam, cy_cam=self.cy_cam, cz_cam=self.cz_cam, size1=MARKER_SIDE1_SIZE, size2=MARKER_SIDE2_SIZE)
    def __str__(self):
        return "color: {}\n  coords: {} {} {}".format(self.color, str(self.cx_cam), str(self.cy_cam), str(self.cz_cam))

# Параметры камеры
cameraMatrix = np.zeros((3, 3), dtype="float64")
distCoeffs = np.zeros((8, 1), dtype="float64")
has_cam_info = False
def camera_info_clb(msg):
    global has_cam_info, cameraMatrix, distCoeffs 
    if not has_cam_info:
        # конвертация параметров камеры из формата ROS в OpenCV
        has_cam_info = True
        distCoeffs = np.array(msg.D, dtype="float64")
        cameraMatrix = np.reshape(np.array(msg.K, dtype="float64"), (3, 3))


def get_color_objs(image, hsv, color_params):
    """
    Обработка изображения для определенного цвета
    """
    debug_out = image.copy()

    mask = cv2.inRange(hsv, color_params[0], color_params[1])
    if len(color_params) == 4:
        mask = cv2.bitwise_or(mask,  cv2.inRange(hsv, color_params[2], color_params[3]))
    # thresh = cv2.threshold(mask, 80, 255, cv2.THRESH_BINARY)[1]
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
    cnts = [i for i in cnts if cv2.contourArea(i) > OBJ_S_THRESH]
    # obj_count = len(cnts)
    debug_out = cv2.bitwise_and(image, image, mask=mask)
    return cnts, debug_out

def img_colision_check(pnts, offset, image_shape=(240, 320, 3)):
    minx1 = pnts[:, 0].min() > offset[0]
    miny1 = pnts[:, 1].min() > offset[1]
    minx2 = image_shape[1] - pnts[:, 0].max() > offset[0]
    miny2 = image_shape[0] - pnts[:, 1].max() > offset[1]
    return minx1 and minx2 and miny1 and miny2

# def get_color_rects(cnts, color_name, image_shape=(240, 320, 3)):
#     """
#     Фильтрация контуров
#     """
#     result = []
#     for cnt in cnts:
#         approx = cv2.approxPolyDP(cnt, 0.05 * cv2.arcLength(cnt, True), True)
#         rect = cv2.minAreaRect(cnt)
#         # print(rect)
#         if len(approx) == 4 and abs(1 - rect[1][0] / (rect[1][1] + 1e-7)) < 0.2:
#             points_img = np.array([np.array(p[0]) for p in approx]) # ?
#             if img_colision_check(points_img, OFFSET,image_shape=image_shape):
#                 M = cv2.moments(cnt)
#                 cX = int((M["m10"] / (M["m00"] + 1e-7)))
#                 cY = int((M["m01"] / (M["m00"] + 1e-7)))
#                 result.append(ColorRect(color=color_name, cx_img=cX, cy_img=cY, points_img=points_img))
#     return result

def get_color_rects_circles(cnts, color_name, image_shape=(240, 320, 3)):
    """
    Фильтрация контуров
    """
    result = []
    circles = []
    for cnt in cnts:
        approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        box_area = cv2.contourArea(box)  + 1e-7
        c_area = cv2.contourArea(cnt)  + 1e-7
        # print(rect)
        if len(approx) == 4 and abs(1 - rect[1][0] / (rect[1][1] + 1e-7)) < 0.15 and abs( 1 - c_area / box_area) < 0.2:
            # points_img = np.array([np.array(p[0]) for p in approx]) # ?
            points_img = box
            if img_colision_check(points_img, OFFSET, image_shape=image_shape):
                M = cv2.moments(cnt)
                cX = int((M["m10"] / (M["m00"] + 1e-7)))
                cY = int((M["m01"] / (M["m00"] + 1e-7)))
                result.append(ColorRect(color=color_name, cx_img=cX, cy_img=cY, points_img=points_img))
        elif len(approx) > 4 and abs(1 - rect[1][0] / (rect[1][1] + 1e-7)) < 0.2 and color_name in ["green", "yellow", "blue"]:
            # elp = cv2.fitEllipse(cnt)
            
            points_img = box
            if img_colision_check(points_img, OFFSET, image_shape=image_shape):
                M = cv2.moments(cnt)
                cX = int((M["m10"] / (M["m00"] + 1e-7)))
                cY = int((M["m01"] / (M["m00"] + 1e-7)))
                circles.append(ColorRect(color=color_name, cx_img=cX, cy_img=cY, points_img=points_img))
    return result, circles


def draw_cnts_colors(image, cnts, color_name, t = 1):
    """
    Отрисовка контуров на итоговом изображении
    """
    for cnt in cnts:
        # M = cv2.moments(cnt)
        # cX = int((M["m10"] / (M["m00"] + 1e-7)))
        # cY = int((M["m01"] / (M["m00"] + 1e-7)))
        cv2.drawContours(image, [cnt], -1, colors_p_rgb[color_name], 2)
        
    return image

def draw_color_rect(image, cr, t = 1):
    """
    Отрисовка результата распознования маркеров
    """
    for i, p in enumerate(cr.points_img):
        cv2.circle(image, tuple(p), 5, ((i+1)*(255//4), (i+1)*(255//4), (i+1)*(255//4)), -1)
    cv2.circle(image, (cr.cx_img, cr.cy_img), 5, colors_p_rgb[cr.color], -1)
    if t:
        cv2.rectangle(image,(cr.cx_img,cr.cy_img-15),(cr.cx_img+75,cr.cy_img+5),(255,255,255),-1)
        cv2.putText(image, type_mapping[cr.color], (cr.cx_img, cr.cy_img),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors_p_rgb[cr.color], 1, cv2.LINE_AA)
    else:
        cv2.putText(image, cr.color, (cr.cx_img, cr.cy_img),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors_p_rgb[cr.color], 2, cv2.LINE_AA)
    return image

def draw_color_circle(image, cr, t = 1):
    """
    Отрисовка результата распознования зон посадки 
    """
    for i, p in enumerate(cr.points_img):
        cv2.circle(image, tuple(p), 5, ((i+1)*(255//4), (i+1)*(255//4), (i+1)*(255//4)), -1)
    cv2.circle(image, (cr.cx_img, cr.cy_img), 5, colors_p_rgb[cr.color], -1)
    if t:
        cv2.rectangle(image,(cr.cx_img,cr.cy_img-15),(cr.cx_img+75,cr.cy_img+5),(255,255,255),-1)
        cv2.putText(image, "LANDING_ZONE", (cr.cx_img, cr.cy_img),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors_p_rgb[cr.color], 1, cv2.LINE_AA)
    else:
        cv2.putText(image, "LANDING_ZONE", (cr.cx_img, cr.cy_img),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors_p_rgb[cr.color], 2, cv2.LINE_AA)
    return image

def get_rect_pose(rect, op, cM, dC):
    """
    Расчет позиции маркера относительно камеры
    """
    # print("shapes", op.shape, rect.points_img.shape)
    retval, rvec, tvec = cv2.solvePnP(np.array(op, dtype="float32"), np.array(rect.points_img, dtype="float32"), cM, dC)
    return ColorRectMarker_p(cx_cam=tvec[0][0], cy_cam=tvec[1][0], cz_cam=tvec[2][0]).fromColorRect(rect)

def img_clb(msg):
    global has_cam_info, cameraMatrix, distCoeffs, markers_arr_pub
    image = bridge.imgmsg_to_cv2(msg, "bgr8")
    # print(image.shape)
    # image[:, :, 2] = np.clip(image[:, :, 2]*0.7, 0, 255)
    # image[:, :, 1] = np.clip(image[:, :, 1]*1.2, 0, 255)
    # image[:, :, 0] = np.clip(image[:, :, 0]*1.5, 0, 255)
    out = image.copy()
    # image = cv2.medianBlur(image,3)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    cv2.imshow("image", image)
    result_in_img_frame = [] # ColorRect
    circles_in_img_frame = []
    for c_name in ["blue", "yellow", "green", "red", "brown"]:
        cnts, d_img = get_color_objs(image, hsv, colors_p_hsv[c_name])
        draw_cnts_colors(out, cnts, c_name)
        # result_in_img_frame += get_color_rects(cnts, c_name)
        k = get_color_rects_circles(cnts, c_name)
        result_in_img_frame += k[0]
        circles_in_img_frame += k[1]
        cv2.imshow(c_name, d_img)
    for i in result_in_img_frame:
        draw_color_rect(out, i)
    for i in circles_in_img_frame:
        draw_color_circle(out, i)
    # cv2.imshow("out", out)
    result = []
    circles = []
    if has_cam_info:
        for r in result_in_img_frame:
            result.append(get_rect_pose(r, objectPoint, cameraMatrix, distCoeffs))
        for c in circles_in_img_frame:
            circles.append(get_rect_pose(c, objectPoint_circles, cameraMatrix, distCoeffs))
        # if len(result) > 0:
        #     print("RES: \n " + "\n ".join(map(str, result)))
        # if len(circles) > 0:
        #     print("circles: \n " + "\n ".join(map(str, circles)))
    # cv2.waitKey(1)
    # Отправка результатов распознования 
    markers_arr_pub.publish(ColorRectMarkerArray(header=Header(stamp=rospy.Time.now(), frame_id="color_marker_cam"), markers=[r.toMsg() for r in result]))
    circles_arr_pub.publish(ColorRectMarkerArray(header=Header(stamp=rospy.Time.now(), frame_id="color_marker_cam"), markers=[r.toMsg() for r in circles]))
    image_pub.publish(bridge.cv2_to_imgmsg(out, "bgr8"))
    cv2.waitKey(1)


image_sub = rospy.Subscriber(
    "image_raw", Image, img_clb)

camera_info_sub = rospy.Subscriber(
    "camera_info", CameraInfo, camera_info_clb)

rospy.spin()
