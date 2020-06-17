#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# tested on python3
# Иморт всего что надо
import math
import rospy
import time
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

"""
Инициализация ноды ROS
"""
rospy.init_node('team_name_color_node', anonymous=True)

"""
Publisher`ы для работы с ROS
"""
image_pub = rospy.Publisher("/color/debug_img", Image)
image_pub_red = rospy.Publisher("/color/debug_img_red", Image)
image_pub_yellow = rospy.Publisher("/color/debug_img_yellow", Image)
image_pub_blue = rospy.Publisher("/color/debug_img_blue", Image)

blue_area_pub = rospy.Publisher("/color/blue_area", Int32)
blue_count_pub = rospy.Publisher("/color/blue_count", Int32)

"""
Параметры цветов
"""
colors_p_hsv = {
    "yellow": (np.array([8,  60,  60]), np.array([35,  255, 255])),
    "red": (np.array([160, 80,  80]), np.array([255, 255, 255])),
    "red2": (np.array([0, 80,  80]), np.array([8, 255, 255])),
    "blue": (np.array([90,  70,  70]), np.array([160, 255, 255]))
}
colors_p_rgb = {
    "yellow": [0,  200,  200],
    "red": [0, 0, 255],
    "blue": [255, 0, 0]
}


"""
Параметры распознования
"""
OBJ_S_THRESH = 150
IMSHOW_ENB = True  # на дроне False

'''
def most_frequent(arr):
    """

    """
    try:
        return max(set(arr), key=arr.count) 
    except:
        return "none"
'''

def get_coords():
    pass

def get_color_objs(image, hsv, color_params, dop_cp=[]):
    """
    Обработка изображения для определенного цвета
    """
    debug_out = image.copy()

    mask = cv2.inRange(hsv, color_params[0], color_params[1])
    if len(dop_cp) > 0:
        mask = cv2.bitwise_or(mask,  cv2.inRange(hsv, dop_cp[0], dop_cp[1]))
    # thresh = cv2.threshold(mask, 80, 255, cv2.THRESH_BINARY)[1]
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
    cnts = [i for i in cnts if cv2.contourArea(i) > OBJ_S_THRESH]
    obj_count = len(cnts)
    debug_out = cv2.bitwise_and(image, image, mask=mask)

    # Вариант 1
    # area = np.sum(mask) / 255
    # Вариант 2
    area = sum(map(cv2.contourArea, cnts))

    return debug_out, area, obj_count, cnts


def draw_cnts_colors(image, cnts, color_name, color):
    """
    Отрисовка контуров на итоговом изображении
    """
    for cnt in cnts:
        M = cv2.moments(cnt)
        cX = int((M["m10"] / (M["m00"] + 1e-7)))
        cY = int((M["m01"] / (M["m00"] + 1e-7)))
        cv2.drawContours(image, [cnt], -1, color, 2)
        cv2.putText(image, color_name, (cX, cY),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2, cv2.LINE_AA)
    return image


def reg_color(image):
    """
    Функция распознования цветов
    Output
    DEBUG Image, RED image, YELLOW image, BLUE image, BLUE area, BLUE obj count
    """
    debug_main = image.copy()
    ratio = 1
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    results = {}
    for c in ["red", "yellow", "blue"]:
        if c == "red":
            results[c] = tuple(get_color_objs(image, hsv, colors_p_hsv[c], dop_cp=colors_p_hsv[c + "2"]))
        else:
            results[c] = tuple(get_color_objs(image, hsv, colors_p_hsv[c]))

    # get_color_objs

    # red, yellow, blue, n, s = waitDataColor(image)
    # 
    # cv2.imshow('red', red)
    # cv2.imshow('yellow', yellow)
    # cv2.imshow('blue', blue)
    # draw_cnts_colors()
    for i in results.keys():
        res = results[i]
        draw_cnts_colors(debug_main, results[i][3], i, colors_p_rgb[i])

    cv2.putText(debug_main, str("Count BLUE: " + str(results["blue"][2]) + " Area BLUE: " + str(results["blue"][1])), (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2, cv2.LINE_AA)

    if IMSHOW_ENB:
        for i in results.keys():
            cv2.imshow(i, results[i][0])

        cv2.imshow('debug_main', debug_main)
        cv2.waitKey(1)
    

    return debug_main, results["red"][0], results["yellow"][0], results["blue"][0], results["blue"][1], results["blue"][2]


def img_clb(data):
    """
    Callback фунция для работы с ROS
    """
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    image, red, yellow, blue, s, n = reg_color(cv_image)
    print('Number of BLUE figures:', n, 'Area of all BLUE figures:', s)
    
    """
    Отправка изображений в топики
    """
    image_pub.publish(bridge.cv2_to_imgmsg(image, "bgr8"))
    image_pub_red.publish(bridge.cv2_to_imgmsg(red, "bgr8"))
    image_pub_yellow.publish(bridge.cv2_to_imgmsg(yellow, "bgr8"))
    image_pub_blue.publish(bridge.cv2_to_imgmsg(blue, "bgr8"))

    """
    Отправка результатов распознования синих объектов
    """
    blue_count_pub.publish(int(n))
    blue_area_pub.publish(int(s))

# cap = cv2.VideoCapture(0)


bridge = CvBridge()


"""
Subscriber для приема данных с камеры
"""
image_sub = rospy.Subscriber(
    "/main_camera/image_raw", Image, img_clb)


rospy.spin()

# while cv2.waitKey(1) != 27:
#     ret, image = cap.read()
#     red, yellow, blue, n, s = waitDataColor(image)
#     print('Number of figures:', n, 'Area of all figures:', s)
#     cv2.imshow('red', red)
#     cv2.imshow('yellow', yellow)
#     cv2.imshow('blue', blue)
#     cv2.imshow('image', image)
