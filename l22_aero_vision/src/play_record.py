#! /usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
# from std_msgs.msg import Int32
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node("simple_python_PLAYER")

bridge = CvBridge()
cap = cv2.VideoCapture("/home/vasily/Videos/LOGS/LOG_IMAGE_RAW_real_drone_1_1.avi")
image_pub = rospy.Publisher("/main_camera/image_raw_throttled", Image)
# cap.set()
cap.set(cv2.CAP_PROP_FRAME_WIDTH,320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,240)
# cap.set(cv2.CAP_PROP_FPS, 25)
rospy.sleep(0.5)
# r = rospy.Rate(40)
# cap.open(0)
print("TEST CODE")
ret, frame = cap.read()
print(ret)
r = rospy.Rate(5)
while not rospy.is_shutdown():
    ret, frame = cap.read()
    print(ret)
    if ret == False:
        break 
    image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
    r.sleep()
cap.release()