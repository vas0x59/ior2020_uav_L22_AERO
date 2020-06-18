#! /usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Int32, Header, Float32
from cv_bridge import CvBridge, CvBridgeError
import os
import time
rospy.init_node('l22_aero_LOGGER', anonymous=True)
fourcc = cv2.VideoWriter_fourcc(*'XVID')


UPDATE_RATE = 5
if not os.path.exists(os.environ['HOME']+"/L22_AERO_LOG"):
    os.mkdir(os.environ['HOME']+"/L22_AERO_LOG")

video_writer = cv2.VideoWriter(os.environ['HOME']+"/L22_AERO_LOG/LOG_IMAGE_RAW_time_" + str(int(time.time())) + ".avi", fourcc, UPDATE_RATE, (320, 240))
r = rospy.Rate(UPDATE_RATE)


image_raw_frame = np.zeros((240, 320, 3), dtype="uint8")


bridge = CvBridge()

def img_clb(msg):
    global image_raw_frame
    image_raw_frame = bridge.imgmsg_to_cv2(msg, "bgr8")

image_sub = rospy.Subscriber(
    "image_raw", Image, img_clb)

rospy.sleep(1)
try:
    while True:
        video_writer.write(image_raw_frame)
        r.sleep()

except KeyboardInterrupt:
    video_writer.release()
    exit()




