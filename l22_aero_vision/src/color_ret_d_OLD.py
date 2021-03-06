#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# tested on python3
# Иморт всего что надо
import math
import rospy
import time
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Int32, Header, Float32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from l22_aero_vision.msg import ColorMarker, ColorMarkerArray

rospy.init_node('l22_aero_color_node', anonymous=True)
bridge = CvBridge()

markers_arr_pub = rospy.Publisher("/l22_aero_color/markers", ColorMarkerArray)

# colors_p_hsv = {
#     "yellow": (np.array([8,  60,  60]), np.array([35,  255, 255])),
#     "red": (np.array([160, 80,  80]), np.array([255, 255, 255]), np.array([0, 80,  80]), np.array([8, 255, 255])),
#     "blue": (np.array([161,  56,  109]), np.array([181, 126, 151])),
#     "green": (np.array([90,  70,  70]), np.array([160, 255, 255])),
#     "brown": (np.array([160, 80,  80]), np.array([255, 255, 255]), np.array([0, 80,  80]), np.array([8, 255, 255]))
# }
colors_p_hsv = {
    'blue': (np.array([72, 121, 67]), np.array([180, 255, 255])),
    'green': (np.array([43, 121, 67]), np.array([116, 255, 255])),
    'yellow': (np.array([15, 121, 67]), np.array([37, 255, 255])),
    'red': (np.array([0, 121, 67]), np.array([5, 255, 255])),
    'brown': (np.array([5, 121, 67]), np.array([24, 255, 255]))
}
colors_p_rgb = {
    "yellow": [0,  200,  200],
    "red": [0, 0, 255],
    "blue": [255, 0, 0],
    "green": [0, 255, 0],
    "brown": [165, 42, 42]
}

MARKER_SIDE1_SIZE = 0.06 # in m
MARKER_SIDE2_SIZE = 0.09 # in m
OBJ_S_THRESH = 150

objectPoint = np.array([(-MARKER_SIDE1_SIZE / 2, -MARKER_SIDE2_SIZE / 2, 0), (MARKER_SIDE1_SIZE / 2, -MARKER_SIDE2_SIZE / 2, 0), 
                        (MARKER_SIDE1_SIZE / 2, MARKER_SIDE2_SIZE / 2, 0), (-MARKER_SIDE1_SIZE / 2, MARKER_SIDE2_SIZE / 2, 0)])
# print("objectPoint shape:", objectPoint.shape)
class Color:
    def __init__(self, cx_img=0, cy_img=0, color="none", points_img=[]):
        self.cx_img = cx_img
        self.cy_img = cy_img
        self.color = color
        self.points_img = points_img
class ColorMarker_p:
    def __init__(self, cx_img=0, cy_img=0, color="none", points_img=[], cx_cam=0, cy_cam=0, cz_cam=0):
        self.cx_img = cx_img
        self.cy_img = cy_img
        self.color = color
        self.points_img = points_img
        self.cx_cam = cx_cam
        self.cy_cam = cy_cam
        self.cz_cam = cz_cam
    def fromColor(self, cr):
        self.cx_img = cr.cx_img
        self.cy_img = cr.cy_img
        self.color = cr.color
        self.points_img = cr.points_img
        return self
    def toMsg(self):
        return ColorMarker(color=self.color, cx_img=self.cx_img, cy_img=self.cy_img, cx_cam=self.cx_cam, cy_cam=self.cy_cam, cz_cam=self.cz_cam)
    def __str__(self):
        return "color: {}\n  coords: {} {} {}".format(self.color, str(self.cx_cam), str(self.cy_cam), str(self.cz_cam))

cameraMatrix = np.zeros((3, 3), dtype="float64")
distCoeffs = np.zeros((8, 1), dtype="float64")
has_cam_info = False
def camera_info_clb(msg: CameraInfo):
    global has_cam_info, cameraMatrix, distCoeffs 
    if not has_cam_info:
        has_cam_info = True
        distCoeffs = np.array(msg.D, dtype="float64")
        cameraMatrix = np.reshape(np.array(msg.K, dtype="float64"), (3, 3))


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
        approx = cv2.approxPolyDP(cnt, 0.1 * cv2.arcLength(cnt, True), True)
        if len(approx) == 4:
            points_img = np.array([np.array(p[0]) for p in approx]) # ?
            M = cv2.moments(cnt)
            cX = int((M["m10"] / (M["m00"] + 1e-7)))
            cY = int((M["m01"] / (M["m00"] + 1e-7)))
            result.append(Color(color=color_name, cx_img=cX, cy_img=cY, points_img=points_img))
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
def draw_color_rect(image, cr: Color):
    for i, p in enumerate(cr.points_img):
        cv2.circle(image, tuple(p), 5, ((i+1)*(255//4), (i+1)*(255//4), (i+1)*(255//4)), -1)
    cv2.circle(image, (cr.cx_img, cr.cy_img), 5, colors_p_rgb[cr.color], -1)
    return image

def get_rect_pose(rect, op, cM, dC) -> ColorMarker_p:
    # print("shapes", op.shape, rect.points_img.shape)
    retval, rvec, tvec = cv2.solvePnP(np.array(op, dtype="float64"), np.array(rect.points_img, dtype="float64"), cM, dC)
    return ColorMarker_p(cx_cam=tvec[0][0], cy_cam=tvec[1][0], cz_cam=tvec[2][0]).fromColor(rect)

def img_clb(msg: Image):
    global has_cam_info, cameraMatrix, distCoeffs, markers_arr_pub
    image = bridge.imgmsg_to_cv2(msg, "bgr8")
    # image[:, :, 2] = np.clip(image[:, :, 2]*0.7, 0, 255)
    # image[:, :, 1] = np.clip(image[:, :, 1]*1.2, 0, 255)
    # image[:, :, 0] = np.clip(image[:, :, 0]*1.5, 0, 255)
    out = image.copy()
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    result_in_img_frame = [] # Color
    for c_name in ["blue"]:
        cnts, d_img = get_color_objs(image, hsv, colors_p_hsv[c_name])
        draw_cnts_colors(out, cnts, c_name)
        result_in_img_frame += get_color_rects(cnts, c_name)
    for i in result_in_img_frame:
        draw_color_rect(out, i)
    cv2.imshow("out", out)
    result = []
    if has_cam_info:
        for r in result_in_img_frame:
            result.append(get_rect_pose(r, objectPoint, cameraMatrix, distCoeffs))
        if len(result) > 0:
            print("RES: \n " + "\n ".join(map(str, result)))
    cv2.waitKey(1)

    markers_arr = ColorMarkerArray(header=Header(stamp=rospy.Time.now(), frame_id="color_marker_cam"), markers=[r.toMsg() for r in result])
    markers_arr_pub.publish(markers_arr)


image_sub = rospy.Subscriber(
    "/main_camera/image_raw", Image, img_clb)

camera_info_sub = rospy.Subscriber(
    "/main_camera/camera_info", CameraInfo, camera_info_clb)


rospy.spin()
