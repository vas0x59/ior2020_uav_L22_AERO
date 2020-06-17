# -*- coding: utf-8 -*-

import math
import time
import cv2
import numpy as np
import rospy
import tf

try:
    from clover           import srv
except:
    from clever           import srv

from std_srvs.srv         import Trigger
from mavros_msgs.srv      import CommandBool
from sensor_msgs.msg      import Image
from std_msgs.msg         import String
from pyzbar               import pyzbar
from cv_bridge            import CvBridge
import sys
import threading

sys.path.append('/home/dmitrii/catkin_ws/src/ior2020_uav_L22_AERO')
from l22_aero_vision.msg  import ColorRectMarker
from l22_aero_vision.msg  import ColorRectMarkerArray
from l22_aero_vision.src.tools.tf_tools import *


Z_TOL = 0.5
TOLERANCE_COORDS = 0.4 #in meters
COORDS_UPDATE_RATE = 1
# ARUCO_TELEM_RATE = 5

coordinates = {
    'water': [],
    'pastures': [],
    'seed': [],
    'potato': [],
    'soil': []
}

type_mapping = {
    'blue': 'water',
    'green': 'pastures',
    'yellow': 'seed',
    'red': 'potato',
    'brown': 'soil'
}

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)


# global_telem_body = get_telemetry_clever(frame_id="body")
# global_telem_aruco = get_telemetry(frame_id="aruco_map")
# global_telem_navigate_target = get_telemetry(frame_id="navigate_target")
# def get_telem_thread_func():
#     global global_telem_aruco, global_telem_navigate_target
#     r2 = rospy.Rate(ARUCO_TELEM_RATE*2)
#     while True:
#         global_telem_aruco = get_telemetry(frame_id="aruco_map")
#         r2.sleep()
#         global_telem_navigate_target = get_telemetry(frame_id="navigate_target")
#         r2.sleep()
# get_telem_thread =  threading.Thread(target=get_telem_thread_func)
# get_telem_thread.daemon = True
# get_telem_thread.start()



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
    # global global_telem_aruco
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
    # global global_telem_navigate_target
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
    rospy.sleep(5)
    arming(False)

class ColorRectMarkerMap:
    def __init__(self, cx_map=0, cy_map=0, cz_map=0, color="none"):
        self.cx_map = cx_map
        self.cy_map = cy_map
        self.cz_map = cz_map
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
        self.cv_image = None
        self.image_sub = rospy.Subscriber('main_camera/image_raw', Image, self.image_callback)
        self.qr_pub = rospy.Publisher('/qr_debug', Image, queue_size=1)
        self.coords_sub = sub = rospy.Subscriber("/l22_aero_color/markers", ColorRectMarkerArray, self.markers_arr_clb)
        self.result = []

        self.coords_thread = threading.Thread(target=self.coords_thread_func)
        self.coords_thread.daemon = True
        # self.coords_thread.start()
        
        

    def transform_marker(self, marker, frame_to="aruco_map"):# -> ColorRectMarkerMap:
        cx_map = 0
        cy_map = 0
        cz_map = 0
        cx_map, cy_map, cz_map, _ = transform_xyz_yaw(
            marker.cx_cam, marker.cy_cam, marker.cz_cam, 0, "main_camera_optical", frame_to, listener)
        return ColorRectMarkerMap(color=marker.color, cx_map=cx_map, cy_map=cy_map, cz_map=cz_map)

    def markers_arr_clb(self, msg):#: ColorRectMarkerArray):
        self.result = []
        for marker in msg.markers:
            self.result.append(self.transform_marker(marker, frame_to="aruco_map"))
        # self.coordsFunc()
        #if len(self.result) > 0:
            #print("RES: \n " + "\n ".join(map(str, self.result)))

    def image_callback(self, data):
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
        # global Z
        '''
        '''
        # arr = [[color1, x1, y1, z1], [color2, x2, y2, z2]]
        global coordinates
        # Z = get_telemetry_aruco().z
        for i in range(len(self.result)):
            if self.result[i].color not in coordinates:
                color = type_mapping[self.result[i].color]
            else:
                color = self.result[i].color
            # if (self.result[i].cz_map - Z) < Z_TOL:
            tempCoords = (self.result[i].cx_map, self.result[i].cy_map)
            if tempCoords[0] < 0 or tempCoords[1] < 0: continue
            if len(coordinates[color]) == 0:
                coordinates[color].append(tempCoords)
            else:
                for j in range(len(coordinates[color])):
                    if self.distance(coordinates[color][j], tempCoords) <= TOLERANCE_COORDS:
                        coordinates[color][j] = self.average(tempCoords, coordinates[color][j])
                        break
                else:
                    coordinates[color].append(tempCoords)
        self.result = []
    def coords_thread_func(self):
        r = rospy.Rate(COORDS_UPDATE_RATE)
        while True:
            self.coordsFunc()
            r.sleep()


    def waitDataQR(self):
        '''
        Функция для распознавания QR-кодов
        '''
        arr = []
        for _ in range(3):
            gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
            barcodes = pyzbar.decode(gray)
            for barcode in barcodes:
                (x, y, w, h) = barcode.rect
                self.barcodeData = barcode.data.decode("utf-8")
                xc = x + w/2
                yc = y + h/2
                self.cv_image = cv2.circle(self.cv_image, (int(xc), int(yc)), 15, (0, 0, 0), 30)
                arr.append(self.barcodeData)
                self.qr_pub.publish(self.bridge.cv2_to_imgmsg(self.cv_image, 'bgr8'))
            rospy.sleep(0.3)
        if len(arr) == 0: return ['none']
        return self.most_frequent(arr)
    
rc = Recognition()
#rospy.spin()



z = 1.5
FIELD_LENGTH = 3.9 #in meters
deltaX = 0.5 #in meters
deltaY = 0.3 #in meters
betweenX = 3



i, count = 0, 0
points = []

def getAdditionalPoints(coord1, coord2, parts):
    return zip(np.linspace(coord1[0], coord2[0], parts + 1), np.linspace(coord1[1], coord2[1], parts + 1))

while i <= FIELD_LENGTH:
    j = 0
    while j <= FIELD_LENGTH:
        if count % 2 == 0:
            points.append((i, j))
        else:
            points.append((i, FIELD_LENGTH-j))
        j += deltaY
    d = j - FIELD_LENGTH
    if d > 0: j -= d
    if count % 2 == 0:
        points += list(getAdditionalPoints((i, j), (i + deltaX, j), betweenX))
    else:
        points += list(getAdditionalPoints((i, FIELD_LENGTH - j), (i + deltaX, FIELD_LENGTH-j), betweenX))
    i += deltaX
    count += 1
    

takeoff(z)
navigate_wait(0, 0, 1, yaw = 3.14/2)

qr = rc.waitDataQR()
print(qr)


if qr == 'seed':
    landCoordinates = (0.15, 3.4)
elif qr == 'water':
    landCoordinates = (3.4, 0.15)
else:
    landCoordinates = (3.4, 3.4)



navigate_wait(0, 0, z)

for point in points:
    navigate_wait(x=point[0], y=point[1], z=z)
    rc.coordsFunc()

'''    
landCoordinates = [(0.15, 3.4), (3.4, 0.15), (3.4, 3.4)]

for point in landCoordinates:
    navigate_wait(point[0], point[1], z)
    rospy.sleep(10)
'''
land()

print('WRITING CSV WITH COORDINATES. PLEASE WAIT...')
print(coordinates)

import csv
from time import time

with open('result_'+str(time())+'.csv', 'w') as f:
    writer = csv.writer(f)
    writer.writerow(["Sector", "Type", "x", "y"])
    arr = []
    for key in coordinates:
        for j in range(len(coordinates[key])):
            x = coordinates[key][j][0]
            y = coordinates[key][j][1]
            if x < FIELD_LENGTH/2 and y < FIELD_LENGTH/2:
                arr.append(['C', key, x, y])
            elif x < FIELD_LENGTH/2 and y >= FIELD_LENGTH/2:
                arr.append(['A', key, x, y])
            elif x >= FIELD_LENGTH/2 and y < FIELD_LENGTH/2:
                arr.append(['D', key, x, y])
            elif x >= FIELD_LENGTH/2 and y >= FIELD_LENGTH/2:
                arr.append(['B', key, x, y])
    arr.sort(key = lambda x: x[0])
    writer.writerows(arr)

print('DONE')
