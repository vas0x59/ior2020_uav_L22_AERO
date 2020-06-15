#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# tested on python3
# Иморт всего что надо
import math
import rospy
import time
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from l22_aero_vision.msg import ColorRectMarker

rospy.init_node('l22_aero_color_node', anonymous=True)
bridge = CvBridge()


colors_p_hsv = {
    "yellow": (np.array([8,  60,  60]), np.array([35,  255, 255])),
    "red": (np.array([160, 80,  80]), np.array([255, 255, 255]), np.array([0, 80,  80]), np.array([8, 255, 255])),
    "blue": (np.array([90,  70,  70]), np.array([160, 255, 255])),
    "green": (np.array([90,  70,  70]), np.array([160, 255, 255])),
    "brown": (np.array([160, 80,  80]), np.array([255, 255, 255]), np.array([0, 80,  80]), np.array([8, 255, 255]))
}
colors_p_rgb = {
    "yellow": [0,  200,  200],
    "red": [0, 0, 255],
    "blue": [255, 0, 0],
    "green": [0, 255, 0],
    "brown": [165, 42, 42]
}

MARKER_SIDE_SIZE = 0.3 # in m
OBJ_S_THRESH = 150

class ColorRect:
    def __init__(self, cx_img=0, cy_img=0, color="none", points_img=[]):
        self.cx_img = cx_img
        self.cy_img = cy_img
        self.color = color
        self.points_img = points_img


cameraMatrix = np.zeros((3, 3), dtype="float64")
distCoeffs = np.zeros((8, 1), dtype="float64")
has_cam_info = False
def camera_info_clb(msg: CameraInfo):
    global has_cam_info, cameraMatrix, distCoeffs 
    if not has_cam_info:
        has_cam_info = True
        distCoeffs = msg.D
        cameraMatrix = np.reshape(np.array(msg.K), (3, 3))


def get_color_objs(image, hsv, color_params):
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

def get_color_rects(cnts, color_name):
    result = []
    for cnt in cnts:
        
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
        if len(approx) == 4:
            points_img = approx # ?
            M = cv2.moments(cnt)
            cX = int((M["m10"] / (M["m00"] + 1e-7)))
            cY = int((M["m01"] / (M["m00"] + 1e-7)))
            result.append(ColorRect(color=color_name, cx_img=cX, cy_img=cY, points_img=points_img))
    return result

def draw_cnts_colors(image, cnts, color_name):
    for cnt in cnts:
        M = cv2.moments(cnt)
        cX = int((M["m10"] / (M["m00"] + 1e-7)))
        cY = int((M["m01"] / (M["m00"] + 1e-7)))
        cv2.drawContours(image, [cnt], -1, colors_p_rgb[color_name], 2)
        cv2.putText(image, color_name, (cX, cY),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors_p_rgb[color_name], 2, cv2.LINE_AA)
    return image
def draw_color_rect(image, cr:ColorRect):
    for i, p in enumerate(cr.points_img):
        cv2.circle(image, p, 5, ((i+1)*(255//4), (i+1)*(255//4), (i+1)*(255//4)), -1)
    cv2.circle(image, (cr.cx_img, cr.cy_img), 5, colors_p_rgb[cr.color], -1)
    return image

def img_clb(msg: Image):
    
    image = bridge.imgmsg_to_cv2(msg, "bgr8")
    out = image.copy()
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    result_in_img_frame = [] # ColorRect
    for c_name in ["red", "yellow", "blue", "green", "brown"]:
        cnts, d_img = get_color_objs(image, hsv, colors_p_hsv[c_name])
        result_in_img_frame += get_color_rects(cnts, c_name)
    for i in result_in_img_frame:
        draw_color_rect(out, i)
        # for i, p in enumerate(cnt):
        #     cv2.circle(out, tuple(p[0]), 5, (0, (i+1)*(255//4)//2, (i+1)*(255//4)), -1)


image_sub = rospy.Subscriber(
    "/main_camera/image_raw", Image, img_clb)

camera_info_sub = rospy.Subscriber(
    "/main_camera/camera_info", CameraInfo, camera_info_clb)



