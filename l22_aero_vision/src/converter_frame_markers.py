import numpy as np
import rospy
# import tf2_geometry_msgs
import tf
from l22_aero_vision.msg import ColorRectMarker, ColorRectMarkerArray
# from l22_aero_vision.src.tool
from l22_aero_vision.src.tools.tf_tools import *

nav_broadcaster = tf.TransformBroadcaster()
# tf_buffer = tf2_ros.Buffer()
# tf_listener = tf2_ros.TransformListener(tf_buffer)
listener = tf.TransformListener()


class ColorRectMarkerMap:
    def __init__(self, cx_map=0, cy_map=0, color="none"):
        self.cx_map = cx_map
        self.cy_map = cy_map
        self.color = color
    def __str__(self):
        return "color: {}\n  coords map: {} {}".format(self.color, str(self.cx_map), str(self.cy_map))


def transform_marker(marker: ColorRectMarker, frame_to="aruco_map") -> ColorRectMarkerMap:
    cx_map = 0
    cy_map = 0
    cx_map, cy_map, _, _ = transform_xyz_yaw(
        marker.cx_cam, marker.cy_cam, marker.cz_cam, 0, "main_camera_optical", frame_to, listener)
    return ColorRectMarkerMap(color=marker.color, cx_map=cx_map, cy_map=cy_map)


def markers_arr_clb(msg: ColorRectMarkerArray):
    result = []
    for marker in msg.markers:
        result.append(transform_marker(marker, frame_to="aruco_map"))
    if len(result) > 0:
        print("RES: \n " + "\n ".join(map(str, result)))


sub = rospy.Subscriber(
    "/l22_aero_color/markers", ColorRectMarkerArray, markers_arr_clb)

rospy.spin()
# tf.transformations.
