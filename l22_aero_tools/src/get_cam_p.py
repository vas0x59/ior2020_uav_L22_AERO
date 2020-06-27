import math
import rospy
import time
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Int32, Header, Float32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import pickle


rospy.init_node("CINFO")
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
        pickle.dump(cameraMatrix, open("cM.pickle", "wb"))
        pickle.dump(distCoeffs, open("dC.pickle", "wb"))



camera_info_sub = rospy.Subscriber(
    "/main_camera/camera_info", CameraInfo, camera_info_clb)

rospy.spin()