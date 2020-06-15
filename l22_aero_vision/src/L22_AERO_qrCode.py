#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import json
from pyzbar import pyzbar

IMSHOW_ENB = True  # на дроне False

"""
Инициализация ноды ROS
"""
rospy.init_node('team_name_qr_node', anonymous=True)

"""
Publisher`ы для работы с ROS
"""
image_pub = rospy.Publisher("/qr/debug_img",Image)
data_pub = rospy.Publisher("/qr/str",String)


def most_frequent(arr):
    try:
        return max(set(arr), key = arr.count)
    except:
        return "none"

def waitDataQR(image):
    out_img = image.copy()
    barcodeData = []
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    """
    Детектирование QR кодов
    """
    barcodes = pyzbar.decode(gray)

    """
    Визуализация
    """
    for barcode in barcodes:
        (x, y, w, h) = barcode.rect
        barcodeData.append(barcode.data.decode("utf-8"))
        xc = int(x + w/2)
        yc = int(y + h/2)
        cv2.rectangle(out_img,(x, y), (x+w, y+h),(0,255,0),3)
        # font = 
        cv2.putText(out_img,barcodeData[-1],(xc,yc), cv2.FONT_HERSHEY_SIMPLEX, 0.5,(0,255,0),2,cv2.LINE_AA)
    return out_img, barcodeData


def img_clb(data):
    """
    Callback фунция для работы с ROS
    """
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    # out_img = cv_image.copy()
    out_img, dd = waitDataQR(cv_image)
    print(sorted(dd))
    if IMSHOW_ENB:
        cv2.imshow('debug_main', out_img)
        cv2.waitKey(1)
    """
    Отправка изображений в топики
    """
    image_pub.publish(bridge.cv2_to_imgmsg(out_img, "bgr8"))

    """
    Отправка результатов распознования 
    """
    data_pub.publish(json.dumps(dd))


"""
класс для конвертации сообщений и изображений
"""
bridge = CvBridge()

"""
Subscriber для приема данных с камеры
"""
image_sub = rospy.Subscriber(
    "/main_camera/image_raw", Image, img_clb)



rospy.spin()

# while cv2.waitKey(1) != 27:
#     ret, image = cap.read()
#     image, data = waitDataQR(image)
#     print(data)
#     cv2.imshow('image', image)