#! /usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rospy
# import tf2_geometry_msgs
import tf
from l22_aero_vision.msg import ColorRectMarker, ColorRectMarkerArray
# from l22_aero_vision.src.tool
from l22_aero_vision.src.tools.tf_tools import *
from visualization_msgs.msg import Marker, MarkerArray
nav_broadcaster = tf.TransformBroadcaster()
# tf_buffer = tf2_ros.Buffer()
# tf_listener = tf2_ros.TransformListener(tf_buffer)
# listener = tf.TransformListener()
rospy.init_node('markers_viz', anonymous=True)

colors_p_rgb = {
    "yellow": [0,  200,  200],
    "red": [0, 0, 255],
    "blue": [255, 0, 0],
    "green": [0, 255, 0],
    "brown": [42, 42, 165]
}

markers_arr_pub = rospy.Publisher("/l22_aero_color/markers_viz", MarkerArray)
circles_arr_pub = rospy.Publisher("/l22_aero_color/circles_viz", MarkerArray)

def markers_arr_clb(msg):
    result = []
    iddd = 0
    for mrk in msg.markers:
        # result.append(transform_marker(marker, frame_to="aruco_map"))
        # cx_map, cy_map, cz_map, _ = transform_xyz_yaw(
        # marker.cx_cam, marker.cy_cam, marker.cz_cam, 0, "main_camera_optical", frame_to, listener)
        marker = Marker()
        marker.header.frame_id = "main_camera_optical"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "color_markers"
        marker.id = iddd
        marker.type =  Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = mrk.cx_cam
        marker.pose.position.y = mrk.cy_cam
        marker.pose.position.z = mrk.cz_cam
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = mrk.size1
        marker.scale.y = mrk.size2
        marker.scale.z = 0.1

        marker.color.a = 0.8

        marker.color.r = colors_p_rgb[mrk.color][2] / 255
        marker.color.g = colors_p_rgb[mrk.color][1] / 255
        marker.color.b = colors_p_rgb[mrk.color][0] / 255

        result.append(marker)
        iddd += 1
    markers_arr_pub.publish(MarkerArray(markers=result))
    # if len(result) > 0:
    #     print("RES: \n " + "\n ".join(map(str, result)))
def circles_arr_clb(msg):
    result = []
    iddd = 0
    for mrk in msg.markers:
        # result.append(transform_marker(marker, frame_to="aruco_map"))
        # cx_map, cy_map, cz_map, _ = transform_xyz_yaw(
        # marker.cx_cam, marker.cy_cam, marker.cz_cam, 0, "main_camera_optical", frame_to, listener)
        marker = Marker()
        marker.header.frame_id = "main_camera_optical"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "color_markers_circles"
        marker.id = iddd
        marker.type =  Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = mrk.cx_cam
        marker.pose.position.y = mrk.cy_cam
        marker.pose.position.z = mrk.cz_cam
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = mrk.size1
        marker.scale.y = mrk.size2
        marker.scale.z = 0.1

        marker.color.a = 0.8

        marker.color.r = colors_p_rgb[mrk.color][2] / 255
        marker.color.g = colors_p_rgb[mrk.color][1] / 255
        marker.color.b = colors_p_rgb[mrk.color][0] / 255

        result.append(marker)
        iddd += 1
    circles_arr_pub.publish(MarkerArray(markers=result))





sub = rospy.Subscriber(
    "/l22_aero_color/markers", ColorRectMarkerArray, markers_arr_clb)
sub2 = rospy.Subscriber(
    "/l22_aero_color/circles", ColorRectMarkerArray, circles_arr_clb)

# tf.transformations.

rospy.spin()