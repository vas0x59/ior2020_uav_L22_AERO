import numpy as np
import rospy
# import tf2_geometry_msgs
import tf
from l22_aero_vision.msg import ColorRectMarker, ColorRectMarkerArray
# from l22_aero_vision.src.tool


nav_broadcaster = tf.TransformBroadcaster()
# tf_buffer = tf2_ros.Buffer()
# tf_listener = tf2_ros.TransformListener(tf_buffer)
listener = tf.TransformListener()


class ColorRectMarkerMap:
    def __init__(self, cx_map=0, cy_map=0, color="none"):
        self.cx_map = cx_map
        self.cy_map = cy_map
        self.color = color


def transform_marker(marker: ColorRectMarker, frame_to="aruco_map") -> ColorRectMarkerMap:
    cx_map = 0
    cy_map = 0
    
    return ColorRectMarkerMap(color=marker.color)


def markers_arr_clb(msg: ColorRectMarkerArray):
    pass





sub = rospy.Subscriber(
    "/l22_aero_color/markers", ColorRectMarkerArray, markers_arr_clb)

# tf.transformations.