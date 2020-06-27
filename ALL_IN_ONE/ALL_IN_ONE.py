#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Импорт библиотек
import math
import rospy
import time
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Int32, Header, Float32
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from threading import Thread
# import tf2
import tf
import tf2_ros
# import tf2_geometry_msgs
import geometry_msgs
import math
# geometry_msgs.msg.
# tf.transformations.
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int16, Bool
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped, PoseStamped
import tf.transformations as t
# from l22_aero_vision.msg import ColorMarker, ColorMarkerArray
# import tf


try:
    from clover           import srv
except:
    from clever           import srv

from std_srvs.srv         import Trigger
from mavros_msgs.srv      import CommandBool
from std_msgs.msg         import String
from pyzbar               import pyzbar
import sys
import threading
import os

from mavros_msgs.srv import CommandBool


def orientation_from_euler(roll, pitch, yaw):
    q = t.quaternion_from_euler(roll, pitch, yaw)
    return orientation_from_quaternion(q)

def euler_from_orientation(o):
    q = quaternion_from_orientation(o)
    return t.euler_from_quaternion(q)

def orientation_from_quaternion(q):
    return Quaternion(*q)
def quaternion_from_orientation(o):
    return o.x, o.y, o.z, o.w

TRANSFORM_TIMEOUT = 1


def transform_xyz_yaw(x, y, z, yaw, framefrom, frameto, listener):
    p = PoseStamped()
    # Pose().fr
    p.header.frame_id = framefrom
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z
    # p.pose.pse
    p.pose.orientation = orientation_from_euler(0, 0, yaw)
    # p.pose
    # p.pose.orientation
    # print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    # pose_local = tf2_geometry_msgs.do_transform_point(p, get_transformation(framefrom, frameto, tf_buffer))
    pose_local = listener.transformPose(frameto, p)
    target_x = pose_local.pose.position.x
    target_y = pose_local.pose.position.y
    target_z = pose_local.pose.position.z
    target_yaw = euler_from_orientation(pose_local.pose.orientation)[2]
    return target_x, target_y, target_z, target_yaw


class VideoRecorder:
    def __init__(self):
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        if not os.path.exists(os.environ['HOME']+"/L22_AERO_LOG"):
            os.mkdir(os.environ['HOME']+"/L22_AERO_LOG")
        self.UPDATE_RATE = 5
        self.video_writer = cv2.VideoWriter(os.environ['HOME']+"/L22_AERO_LOG/LOG_IMAGE_RAW_real_drone.avi", self.fourcc, self.UPDATE_RATE, (320, 240))
        self.image_raw_frame = np.zeros((240, 320, 3), dtype="uint8")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/main_camera/image_raw_throttled", Image, self.img_clb)
        self.stopped = False
    
    def img_clb(self, msg):
        self.image_raw_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def start(self):    
        Thread(target=self.videowriter, args=()).start()
        return self

    def videowriter(self):
        try:
            r = rospy.Rate(self.UPDATE_RATE)
            while not self.stopped:
                self.video_writer.write(self.image_raw_frame)
                r.sleep()
        except KeyboardInterrupt:
            self.video_writer.release()
            self.stopped = True
    def stop(self):
        self.stopped = True
        self.video_writer.release()













result_GLOBAL = [] # ColorMarker_p
circles_GLOBAL = [] # ColorMarker_p


rospy.init_node('l22_aero_color_node', anonymous=True)
bridge = CvBridge()

# markers_arr_pub = rospy.Publisher("/l22_aero_color/markers", ColorMarkerArray)
# circles_arr_pub = rospy.Publisher("/l22_aero_color/circles", ColorMarkerArray)
image_pub = rospy.Publisher("/l22_aero_color/debug_img", Image)
# Параметры цвета маркеров
# colors_p_hsv = {
#     'blue': (np.array([103, 47, 65]), np.array([150, 187, 172])),
#     'green': (np.array([28, 44, 20]), np.array([100, 255, 255])),
#     'yellow': (np.array([14, 100, 104]), np.array([29, 255, 255])),
#     'red': (np.array([151, 134, 99]), np.array([255, 243, 252])),
#     'brown': (np.array([6, 86, 99]), np.array([255, 243, 252]))
# }
'''
colors_p_hsv = {
    'blue': (np.array([72, 121, 67]), np.array([180, 255, 255])),
    'green': (np.array([43, 121, 67]), np.array([116, 255, 255])),
    'yellow': (np.array([15, 121, 67]), np.array([37, 255, 255])),
    'red': (np.array([0, 121, 67]), np.array([5, 255, 255])),
    'brown': (np.array([5, 121, 67]), np.array([24, 255, 255]))
}
'''
colors_p_hsv = {
    'green': (np.array([71, 86, 22]), np.array([88, 255, 255])),
    'yellow': (np.array([13, 63, 74]), np.array([65, 255, 255])),
    'blue': (np.array([94, 88, 63]), np.array([134, 255, 255])),
    'red': (np.array([133, 94, 62]), np.array([241, 255, 255])),
    'brown': (np.array([0, 111, 98]), np.array([12, 255, 255])),
}

colors_p_rgb = {
    "yellow": [0,  200,  200],
    "red": [0, 0, 255],
    "blue": [255, 0, 0],
    "green": [0, 255, 0],
    "brown": [42, 42, 165]
}


# Формат вывода 
type_mapping_1 = {
    'blue': 'N2_water',
    'green': 'N2_pastures',
    'yellow': 'N2_seed',
    'red': 'N2_potato',
    'brown': 'N2_soil'
}

# # Размеры цветных маркеров 
MARKER_SIDE1_SIZE = 0.35 # in m
MARKER_SIDE2_SIZE = 0.35 # in m
CIRCLE_R = 0.2

# Размеры цветных маркеров 
# MARKER_SIDE1_SIZE = 0.3 # in m
# MARKER_SIDE2_SIZE = 0.3 # in m
# CIRCLE_R = 0.15


OBJ_S_THRESH = 350
OFFSET = [61, 35] # pixels

objectPoint = np.array([(-MARKER_SIDE1_SIZE / 2, -MARKER_SIDE2_SIZE / 2, 0), (MARKER_SIDE1_SIZE / 2, -MARKER_SIDE2_SIZE / 2, 0), 
                        (MARKER_SIDE1_SIZE / 2, MARKER_SIDE2_SIZE / 2, 0), (-MARKER_SIDE1_SIZE / 2, MARKER_SIDE2_SIZE / 2, 0)])
objectPoint_circles = np.array([(-CIRCLE_R, -CIRCLE_R, 0), (CIRCLE_R, -CIRCLE_R, 0), 
                        (CIRCLE_R, CIRCLE_R, 0), (-CIRCLE_R, CIRCLE_R, 0)])
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
        return ColorMarker(color=self.color, cx_img=self.cx_img, cy_img=self.cy_img, cx_cam=self.cx_cam, cy_cam=self.cy_cam, cz_cam=self.cz_cam, size1=MARKER_SIDE1_SIZE, size2=MARKER_SIDE2_SIZE)
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
#         approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
#         rect = cv2.minAreaRect(cnt)
#         # print(rect)
#         if len(approx) == 4 and abs(1 - rect[1][0] / (rect[1][1] + 1e-7)) < 0.2:
#             points_img = np.array([np.array(p[0]) for p in approx]) # ?
#             if img_colision_check(points_img, OFFSET,image_shape=image_shape):
#                 M = cv2.moments(cnt)
#                 cX = int((M["m10"] / (M["m00"] + 1e-7)))
#                 cY = int((M["m01"] / (M["m00"] + 1e-7)))
#                 result.append(Color(color=color_name, cx_img=cX, cy_img=cY, points_img=points_img))
#     return result

def get_color_rects_circles(cnts, color_name, image_shape=(240, 320, 3)):
    global OFFSET
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
        if len(approx) == 4 and abs(1 - rect[1][0] / (rect[1][1] + 1e-7)) < 0.15 and abs(c_area / box_area) > 0.85:
            points_img = np.array([np.array(p[0]) for p in approx]) # ?
            # points_img = box
            if img_colision_check(points_img, OFFSET, image_shape=image_shape):
                M = cv2.moments(cnt)
                cX = int((M["m10"] / (M["m00"] + 1e-7)))
                cY = int((M["m01"] / (M["m00"] + 1e-7)))
                result.append(Color(color=color_name, cx_img=cX, cy_img=cY, points_img=points_img))
        elif len(approx) > 4 and abs(1 - rect[1][0] / (rect[1][1] + 1e-7)) < 0.2 and color_name in ["green", "yellow", "blue"] and abs(c_area / box_area) < 0.8:
            # elp = cv2.fitEllipse(cnt)
            
            points_img = box
            if img_colision_check(points_img, OFFSET, image_shape=image_shape):
                M = cv2.moments(cnt)
                cX = int((M["m10"] / (M["m00"] + 1e-7)))
                cY = int((M["m01"] / (M["m00"] + 1e-7)))
                circles.append(Color(color=color_name, cx_img=cX, cy_img=cY, points_img=points_img))
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
        cv2.putText(image, type_mapping_1[cr.color], (cr.cx_img, cr.cy_img),
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
    retval, rvec, tvec = cv2.solvePnP(np.array(op, dtype="float64"), np.array(rect.points_img, dtype="float64"), cM, dC)
    return ColorMarker_p(cx_cam=tvec[0][0], cy_cam=tvec[1][0], cz_cam=tvec[2][0]).fromColor(rect)

def img_clb(msg):
    global has_cam_info, cameraMatrix, distCoeffs, result_GLOBAL, circles_GLOBAL
    image = bridge.imgmsg_to_cv2(msg, "bgr8")
    # print(image.shape)
    # image[:, :, 2] = np.clip(image[:, :, 2]*0.7, 0, 255)
    # image[:, :, 1] = np.clip(image[:, :, 1]*1.2, 0, 255)
    # image[:, :, 0] = np.clip(image[:, :, 0]*1.5, 0, 255)
    out = image.copy()
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    result_in_img_frame = [] # Color
    circles_in_img_frame = []
    for c_name in ["blue", "yellow", "green", "red", "brown"]:
        cnts, d_img = get_color_objs(image, hsv, colors_p_hsv[c_name])
        draw_cnts_colors(out, cnts, c_name)
        # result_in_img_frame += get_color_rects(cnts, c_name)
        k = get_color_rects_circles(cnts, c_name)
        result_in_img_frame += k[0]
        circles_in_img_frame += k[1]
    for i in result_in_img_frame:
        draw_color_rect(out, i)
    for i in circles_in_img_frame:
        draw_color_circle(out, i)
    # cv2.imshow("out", out)
    result_GLOBAL = []
    circles_GLOBAL = []
    if has_cam_info:
        for r in result_in_img_frame:
            result_GLOBAL.append(get_rect_pose(r, objectPoint, cameraMatrix, distCoeffs))
        for c in circles_in_img_frame:
            circles_GLOBAL.append(get_rect_pose(c, objectPoint_circles, cameraMatrix, distCoeffs))
        # if len(result) > 0:
        #     print("RES: \n " + "\n ".join(map(str, result)))
        # if len(circles) > 0:
        #     print("circles: \n " + "\n ".join(map(str, circles)))
    # cv2.waitKey(1)
    # Отправка результатов распознования 
    # markers_arr_pub.publish(ColorMarkerArray(header=Header(stamp=rospy.Time.now(), frame_id="color_marker_cam"), markers=[r.toMsg() for r in result]))
    # circles_arr_pub.publish(ColorMarkerArray(header=Header(stamp=rospy.Time.now(), frame_id="color_marker_cam"), markers=[r.toMsg() for r in circles]))
    image_pub.publish(bridge.cv2_to_imgmsg(out, "bgr8"))


image_sub = rospy.Subscriber(
    "/main_camera/image_raw_throttled", Image, img_clb)

camera_info_sub = rospy.Subscriber(
    "/main_camera/camera_info", CameraInfo, camera_info_clb)

# rospy.spin()

########################################################################
########################################################################
########################################################################
########################################################################
########################################################################
########################################################################
########################################################################
########################################################################
########################################################################
########################################################################
########################################################################
########################################################################
########################################################################
########################################################################
########################################################################
########################################################################


Z_TOL = 0.5
TOLERANCE_COORDS = 0.4 #in meters
COORDS_UPDATE_RATE = 1
# ARUCO_TELEM_RATE = 5

# Словари для координат
coordinates = {
    'water': [],
    'pastures': [],
    'seed': [],
    'potato': [],
    'soil': [],
    'water_land': [],
    'seed_land': [],
    'pastures_land': []
}

type_mapping = {
    'blue': 'water',
    'green': 'pastures',
    'yellow': 'seed',
    'red': 'potato',
    'brown': 'soil'
}

circle_type_mapping = {
    'seed': 'seed_land',
    'pastures': 'pastures_land',
    'water': 'water_land',
    'blue': 'water_land',
    'green': 'pastures_land',
    'yellow': 'seed_land'
}


get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land_serv = rospy.ServiceProxy('land', Trigger)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

nav_broadcaster = tf.TransformBroadcaster()
# tf_buffer = tf2_ros.Buffer()
# tf_listener = tf2_ros.TransformListener(tf_buffer)
listener = tf.TransformListener()




##### 

Z = 0


def navigate_aruco(x=0, y=0, z=0, yaw=float('nan'), speed=0.4,  floor=False):
    '''
    Фукнция для полета до точки без ожидания долета до нее
    '''
    return navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id='aruco_map')

def get_telemetry_aruco():
    '''
    Функция для получения телеметрии
    '''
    telem = get_telemetry(frame_id="aruco_map")
    # Z = telem.z
    return telem

def takeoff(z):
    '''
    Функция для взлета
    '''
    telem = get_telemetry_aruco()
    navigate(z=z, speed=0.3, frame_id="body", auto_arm=True)
    rospy.sleep(2)
    navigate_aruco(x=telem.x, y=telem.y, z=z, speed=0.4, floor=True)

def navigate_wait(x, y, z, yaw=float('nan'), speed=0.2, tolerance=0.13):
    '''
    Фукнция для полета до точки с ожиданием долета до нее
    '''
    navigate_aruco(x=x, y=y, z=z, yaw=yaw, speed=speed)
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id="navigate_target")
        # print(telem.x, telem.y, telem.z)
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def land():
    '''
    Фунцкия для посадки
    '''
    land_serv()
    rospy.sleep(0.79)
    arming(False)

class ColorMarkerMap:
    def __init__(self, cx_map=0, cy_map=0, cz_map=0, cx_img=0, cy_img=0, color="none"):
        self.cx_map = cx_map
        self.cy_map = cy_map
        self.cz_map = cz_map
        self.cx_img = cx_img
        self.cy_img = cy_img
        self.color = color
    def __str__(self):
        return "color: {}\n  coords map: {} {} {}".format(self.color, str(self.cx_map), str(self.cy_map), str(self.cz_map))


class Recognition:
    def __init__(self):
        '''
        Инициализация переменных
        Создание подписчика топика main_camera/image_raw_throttled, который отправляет его в image_callback()
        Создание топика, который публикует изображение
        '''
        self.barcodeData = None
        self.bridge = CvBridge()
        self.cv_image = np.zeros((240, 320, 3), dtype="uint8")
        self.image_sub = rospy.Subscriber('/main_camera/image_raw_throttled', Image, self.image_callback)
        self.qr_pub = rospy.Publisher('/qr_debug', Image, queue_size=1)
        # self.coords_sub = sub = rospy.Subscriber("/l22_aero_color/markers", ColorMarkerArray, self.markers_arr_clb)
        # self.circles_sub = rospy.Subscriber("/l22_aero_color/circles", ColorMarkerArray, self.circles_arr_clb)
        self.result = []
        self.circles = []
    
        # self.coords_thread = threading.Thread(target=self.coords_thread_func)
        # self.coords_thread.daemon = True
        # self.coords_thread.start()

    def transform_marker(self, marker, frame_to="aruco_map"):# -> ColorMarkerMap:
        cx_map = 0
        cy_map = 0
        cz_map = 0
        try:
            cx_map, cy_map, cz_map, _ = transform_xyz_yaw(
                marker.cx_cam, marker.cy_cam, marker.cz_cam, 0, "main_camera_optical", frame_to, listener)
        except (tf.LookupException, tf.ConnectivityException):
            print("TF erro")
        return ColorMarkerMap(color=marker.color, cx_map=cx_map, cy_map=cy_map, cz_map=cz_map)

    def markers_arr_clb(self, msg):
        '''
        Функция для парсинга координат цветных маркеров
        '''
        # self.result = []
        for marker in msg:
            self.result.append(self.transform_marker(marker, frame_to="aruco_map"))

    def circles_arr_clb(self, msg):
        '''
        Функция для парсинга координат точек для посадки из топика
        '''
        # self.circles = []
        for marker in msg:
            self.circles.append(self.transform_marker(marker, frame_to="aruco_map"))

    def image_callback(self, data):
        '''
        Функция для парсинга изображения из топика
        '''
        self.cv_image = cv2.resize(self.bridge.imgmsg_to_cv2(data, 'bgr8'), (320, 240))

    def most_frequent(self, arr):
        '''
        Функция для определения значения, который встречается наибольшее количество раз в массиве
        '''
        return max(set(arr), key = arr.count)

    def distance(self, coord1, coord2):
        '''
        Функция для определения евклидова расстояния
        '''
        return ((coord1[0] - coord2[0])**2 + (coord1[1] - coord2[1])**2)**0.5
    
    def average(self, coord1, coord2):
        '''
        Функция для определения средней точки
        '''
        return ((coord1[0] + coord2[0])/2, (coord1[1] + coord2[1])/2)
    def coordsFunc(self):
        global result_GLOBAL, circles_GLOBAL, coordinates
        # global Z
        '''
        Функция для усреднения координат цветных маркеров
        '''
        self.markers_arr_clb(result_GLOBAL)
        self.circles_arr_clb(circles_GLOBAL)
        print("GLOBAL",result_GLOBAL, circles_GLOBAL)
        # global coordinates
        # Z = get_telemetry_aruco().z
        for i in range(len(self.result)):
            if self.result[i].color not in coordinates:
                color = type_mapping[self.result[i].color]
            else:
                color = self.result[i].color
            # if (self.result[i].cz_map - Z) < Z_TOL:
            tempCoords = (self.result[i].cx_map, self.result[i].cy_map)
            if tempCoords[0] < -1.5 or tempCoords[1] < -1.5: continue
            if len(coordinates[color]) == 0:
                coordinates[color].append(np.array([np.array(tempCoords)]))
            else:
                for j in range(len(coordinates[color])):
                    # print(coordinates[color][j].mean(axis=0), tempCoords, "arr", coordinates[color][j])
                    if self.distance(coordinates[color][j].mean(axis=0), tempCoords) <= TOLERANCE_COORDS:
                        coordinates[color][j] = np.append(coordinates[color][j], np.array([tempCoords]), axis=0)
                        break
                else:
                    coordinates[color].append(np.array([np.array(tempCoords)]))
        self.result = []
        for i in range(len(self.circles)):
            if self.circles[i].color not in coordinates:
                color = circle_type_mapping[self.circles[i].color]
            else:
                color = self.circles[i].color
            tempCoords = [self.circles[i].cx_map, self.circles[i].cy_map]#####################################
            if tempCoords[0] < -1 or tempCoords[1] < -1: continue #DELETE IF NEEDED!
            if len(coordinates[color]) == 0:
                coordinates[color].append(np.array([np.array(tempCoords)]))##################################################
            else:
                for j in range(len(coordinates[color])):
                    if self.distance(coordinates[color][j].mean(axis=0), tempCoords) <= TOLERANCE_COORDS:
                        # coordinates[color][j] = list(self.average(tempCoords, coordinates[color][j])) + [coordinates[color][j][2] + 1] ################################
                        coordinates[color][j] = np.append(coordinates[color][j], np.array([tempCoords]), axis=0)
                        break
                else:
                    coordinates[color].append(np.array([np.array(tempCoords)]))######coordinates[color].append(list(tempCoords) + [1]) #########################################################
        self.circles = []

    def coords_thread_func(self):
        r = rospy.Rate(COORDS_UPDATE_RATE)
        while True:
            self.coordsFunc()
            r.sleep()


    def waitDataQR(self):
        '''
        Функция для распознавания QR-кодов
        '''
        gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        barcodes = pyzbar.decode(gray)
        for barcode in barcodes:
            (x, y, w, h) = barcode.rect
            self.barcodeData = barcode.data.decode("utf-8")
            xc = x + w/2
            yc = y + h/2
            self.cv_image = cv2.circle(self.cv_image, (int(xc), int(yc)), 15, (0, 0, 0), 30)
            self.qr_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, 'bgr8'))
        return self.barcodeData
        
# Создание объекта класса для распознавания
rc = Recognition()
vr = VideoRecorder().start()


z = 1.5
FIELD_LENGTH_X = 2.83 #in meters
FIELD_LENGTH_Y = 2.65 #in meters
deltaX = 0.65 #in meters
deltaY = 0.25 #in meters
betweenX = 3
LANDING_B = 5


i, count = 0.1, 0
points = []

def getAdditionalPoints(coord1, coord2, parts, xyz=0):
    '''
    Создание дополнительных точек между двумя данными
    '''
    if xyz:
        return zip(np.linspace(coord1[0], coord2[0], parts + 1), np.linspace(coord1[1], coord2[1], parts + 1), np.linspace(coord1[2], coord2[2], parts + 1))
    return zip(np.linspace(coord1[0], coord2[0], parts + 1), np.linspace(coord1[1], coord2[1], parts + 1))


# Создание массива с точками для дальнейшего полета по полю (полет по зиг-загу)
while i <= FIELD_LENGTH_X:
    j = 0.1
    while j <= FIELD_LENGTH_Y:
        if count % 2 == 0:
            points.append((i, j))
        else:
            points.append((i, FIELD_LENGTH_Y-j))
        j += deltaY
    d = j - FIELD_LENGTH_Y
    if d > 0: j -= d
    if count % 2 == 0 and i != len(points) - 1:
        points += list(getAdditionalPoints((i, j), (i + deltaX, j), betweenX))
    elif count % 2 != 0 and i != len(points) - 1:
        points += list(getAdditionalPoints((i, FIELD_LENGTH_Y - j), (i + deltaX, FIELD_LENGTH_Y-j), betweenX))
    i += deltaX
    count += 1

if points[-1][0] > FIELD_LENGTH_X:
    points = points[:-3]

# взлет
takeoff(z)
navigate_wait(0.15, 0.15, 1.2)

# распознавание qr-кода
qrs = []
qr = 'seed'
zLower = 1.05

# полет вокруг qr-кода для улучшения распознавания
for (x_new, y_new) in [(0.12, 0.15), (0.18, 0.05), (0.20, 0.05), (0.23, 0.2), (0.2, 0.25), (0.15, 0.15)]:
    navigate_wait(x_new, y_new, zLower)
    qrs.append(rc.waitDataQR())
    rospy.sleep(0.55)

if len(qrs) > 0:
    qr = rc.most_frequent(qrs)

if qr == None: 
    qr = 'seed'
    print(".....")

print(qr)

navigate_wait(0.15, 0.1, z)

# полет по полю
for point in points:
    '''
    if points.index(point) == int(len(points) // 4):
        break
    '''
    navigate_wait(x=point[0], y=point[1], z=z, speed=0.4, yaw=3.14/2.0)
    rospy.sleep(0.3)
    rc.coordsFunc()
    rospy.sleep(0.3)
print("739")

# for (x_new, y_new) in [(0.75*FIELD_LENGTH_X, 0.75*FIELD_LENGTH_Y), (0.75*FIELD_LENGTH_X, FIELD_LENGTH_Y/4), (FIELD_LENGTH_X/4, FIELD_LENGTH_Y/4), (FIELD_LENGTH_X/4, 0.75*FIELD_LENGTH_Y)]:
#     navigate_wait(x_new, y_new, 2.1)
#     rospy.sleep(1)

    
print(coordinates[circle_type_mapping[qr]])
# определение координат для посадки
if len(coordinates[circle_type_mapping[qr]]) == 0:
    landCoordinate = (1, 1)
    print("1, 1")
else:
    landCoordinate = max(coordinates[circle_type_mapping[qr]], key=len).mean(axis=0) ###############################################################
    print("landCoordinate", landCoordinate)
print("746")
# посадка

telem = get_telemetry_aruco()
for (x_new, y_new) in list(getAdditionalPoints((telem.x, telem.y), landCoordinate, 3)):
    navigate_wait(x_new, y_new, z)

navigate_wait(landCoordinate[0], landCoordinate[1], z)
print("749")

print('WRITING CSV WITH COORDINATES')
print(coordinates)

# Создание csv файла с координатами

if not os.path.exists(os.environ['HOME']+"/L22_AERO_LOG"):
    os.mkdir(os.environ['HOME']+"/L22_AERO_LOG")

import csv
# from time import time

with open(os.environ['HOME']+"/L22_AERO_LOG/" + 'L22_AERO_result.csv', 'w') as f:
    writer = csv.writer(f)
    writer.writerow(["Sector", "Type", "x (cm)", "y (cm)"])
    arr = []
    for key in coordinates:
        if key in ['water_land', 'seed_land', 'pastures_land']: continue
        for j in range(len(coordinates[key])):
            point = coordinates[key][j].mean(axis=0)
            x = point[0]
            y = point[1]
            if x < FIELD_LENGTH_X/2 and y < FIELD_LENGTH_Y/2:
                arr.append(['C', key, x*100, y*100])
            elif x < FIELD_LENGTH_X/2 and y >= FIELD_LENGTH_Y/2:
                arr.append(['A', key, x*100, y*100])
            elif x >= FIELD_LENGTH_X/2 and y < FIELD_LENGTH_Y/2:
                arr.append(['D', key, x*100, y*100])
            elif x >= FIELD_LENGTH_X/2 and y >= FIELD_LENGTH_Y/2:
                arr.append(['B', key, x*100, y*100])
    arr.sort(key = lambda x: x[0])
    writer.writerows(arr)
    writer.writerow(['','','',''])
    writer.writerow(['','','',''])
    writer.writerow(['','','',''])
    writer.writerow(['','','',''])
    writer.writerow(['TIME:', str(time.time()), 'TIME:', str(time.time())])
print('CSV SAVED')


telem = get_telemetry_aruco()

last = None

Z_LAND = 0.85

landingPath = list(getAdditionalPoints((landCoordinate[0], landCoordinate[1], z), (landCoordinate[0], landCoordinate[1], Z_LAND), 3, xyz = 1))
print(landingPath)
j = 0
print("756")
markerType = circle_type_mapping[qr]
while j < len(landingPath):
    print(i, j)
    rc.markers_arr_clb(result_GLOBAL)
    rc.circles_arr_clb(circles_GLOBAL)
    print(rc.circles)
    circles_copy = list(rc.circles)
    if len(circles_copy) > 0:
        for i in range(len(circles_copy)):
            if rc.distance((circles_copy[i].cx_map, circles_copy[i].cy_map), landCoordinate) <= 0.6 and circle_type_mapping[circles_copy[i].color] == markerType:
                navigate_wait(circles_copy[i].cx_map, circles_copy[i].cy_map, landingPath[j][2], tolerance=0.15)
                last = list(circles_copy)
                break
        else:
            circles_copy = []
        j += 1
    
    if len(circles_copy) == 0:
        if last == None:
            navigate_wait(landCoordinate[0], landCoordinate[1], 1.5)
        else:
            navigate_wait(circles_copy[-1].cx_map, circles_copy[-1].cy_map, 1.5)

    telem = get_telemetry_aruco()
print("777")
# LANDING SYSTEM
print("markerType_LANDING", markerType)
print("STAGE2")
time_st = time.time()
TIMEOUT_H = 2.85
landing_update_rate = rospy.Rate(3.7)
OFFSET = [2, 2] # pixels

while (time.time() - time_st) < TIMEOUT_H:
    markers = [i for i in circles_GLOBAL if circle_type_mapping[i.color] == markerType]
    print(markers)
    if len(markers) > 0:
        marker = markers[0]
        x_b, y_b, z_b, _ = transform_xyz_yaw(
                marker.cx_cam, marker.cy_cam, marker.cz_cam, 0, "main_camera_optical", "body", listener)
        # nav_broadcaster.sendTransform(
        #     (x_b, y_b, z_b),
        #     tf.transformations.quaternion_from_euler(0, 0, 0),
        #     rospy.Time.now(),
        #     "landing_target",
        #     "body"
        # )
        rospy.sleep(0.087)
        print(x_b, y_b, z_b)
        
        set_position(x=x_b, y=y_b, z=-0.076, frame_id="body")
        if abs(z_b) < 0.2:
            break
    landing_update_rate.sleep()


print("LANDDDDDDDDDDDDDDDD")
land()
vr.stop()
rospy.sleep(1.5)
print("DISARM")
arming(False)
print('DONE')
