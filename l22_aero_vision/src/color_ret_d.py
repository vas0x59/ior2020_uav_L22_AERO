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

cameraMatrix = np.zeros((3, 3), dtype="float64")
distCoeffs = np.zeros((8, 1), dtype="float64")
has_cam_info = False

MARKER_SIDE_SIZE = 0.3 # in m


def camera_info_clb(msg: CameraInfo):
    global has_cam_info, cameraMatrix, distCoeffs 
    if not has_cam_info:
        has_cam_info = True
        distCoeffs = msg.D
        cameraMatrix = np.reshape(np.array(msg.K), (3, 3))


image_sub = rospy.Subscriber(
    "/main_camera/image_raw", Image, img_clb)

camera_info_sub = rospy.Subscriber(
    "/main_camera/camera_info", CameraInfo, camera_info_clb)



