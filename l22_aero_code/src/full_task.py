#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Импорт библиотек
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
import os

from mavros_msgs.srv import CommandBool
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

# sys.path.append('/home/dmitrii/catkin_ws/src/ior2020_uav_L22_AERO')
sys.path.append('/home/pi/catkin_ws/src/ior2020_uav_L22_AERO')
from l22_aero_vision.msg  import ColorMarker
from l22_aero_vision.msg  import ColorMarkerArray
from l22_aero_vision.src.tools.tf_tools import *
import l22_aero_vision.srv 

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

rospy.init_node('flight')



# создаем объекты прокси сервисов
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land_serv = rospy.ServiceProxy('land', Trigger)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
v_set_parametrs = rospy.ServiceProxy('/l22_aero_color/set_parametrs', l22_aero_vision.srv.SetParameters)

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
    def __init__(self, cx_map=0, cy_map=0, cz_map=0, cx_img=0, cy_img=0, color="none", cx_cam=0, cy_cam=0, cz_cam=0):
        self.cx_map = cx_map
        self.cy_map = cy_map
        self.cz_map = cz_map

        self.cx_cam = cx_cam
        self.cy_cam = cy_cam
        self.cz_cam = cz_cam

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
        self.image_sub = rospy.Subscriber('image_raw', Image, self.image_callback)
        self.qr_pub = rospy.Publisher('/qr_debug', Image, queue_size=1)
        self.coords_sub  = rospy.Subscriber("/l22_aero_color/markers", ColorMarkerArray, self.markers_arr_clb)
        self.circles_sub = rospy.Subscriber("/l22_aero_color/circles", ColorMarkerArray, self.circles_arr_clb)
        self.result = []
        self.circles = []
    
        self.coords_thread = threading.Thread(target=self.coords_thread_func)
        self.coords_thread.daemon = True
        # self.coords_thread.start()

    def transform_marker(self, marker, frame_to="aruco_map"):# -> ColorMarkerMap:
        cx_map = 0
        cy_map = 0
        cz_map = 0
        try:
            cx_map, cy_map, cz_map, _ = transform_xyz_yaw(
                marker.cx_cam, marker.cy_cam, marker.cz_cam, 0, "main_camera_optical", frame_to, listener)
        except (tf.LookupException, tf.ConnectivityException):
            print("TF error")
        return ColorMarkerMap(color=marker.color, cx_map=cx_map, cy_map=cy_map, cz_map=cz_map, 
            cx_cam=marker.cx_cam, cy_cam=marker.cy_cam, cz_cam=marker.cz_cam)

    def markers_arr_clb(self, msg):
        '''
        Функция для парсинга координат цветных маркеров
        '''
        self.result = []
        for marker in msg.markers:
            self.result.append(self.transform_marker(marker, frame_to="aruco_map"))

    def circles_arr_clb(self, msg):
        '''
        Функция для парсинга координат точек для посадки из топика
        '''
        self.circles = []
        for marker in msg.markers:
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
        # global Z
        '''
        Функция для усреднения координат цветных маркеров
        '''
        global coordinates
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
# OFFSET = [2, 2] # pixels


while (time.time() - time_st) < TIMEOUT_H:
    markers = [i for i in rc.circles if circle_type_mapping[i.color] == markerType]
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

