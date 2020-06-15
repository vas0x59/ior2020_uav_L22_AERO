import rospy
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
# import tf2

TRANSFORM_TIMEOUT = 1

def get_dist(x1, y1, x2, y2):
    return ((x1-x2) ** 2 + (y1-y2) ** 2)**0.5

def offset_yaw(yaw, zero_yaw):
    itog = yaw
    itog = yaw - zero_yaw
    if (itog > 1.0 * math.pi):
            itog -= 2.0 * math.pi
    if (itog < -1.0 * math.pi):
        itog+= 2.0 * math.pi
    return itog

# def transform_point(transformation, point_wrt_source):
#     # tf2_ros.
#     point_wrt_target = \
#         tf2_geometry_msgs.do_transform_point(PointStamped(point=point_wrt_source),
#             transformation).point
#     return [point_wrt_target.x, point_wrt_target.y, point_wrt_target.z]


# def get_transformation(source_frame, target_frame,tf_buffer,
#                        tf_cache_duration=2.0):
#     # tf_buffer = tf2_ros.Buffer(rospy.Duration(tf_cache_duration))
#     # tf2_ros.TransformListener(tf_buffer)

#     # get the tf at first available time
#     try:
#         transformation = tf_buffer.lookup_transform(target_frame,
#                 source_frame, rospy.Time(0), rospy.Duration(0.1))
#     except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
#             tf2_ros.ExtrapolationException):
#         rospy.logerr('Unable to find the transformation from %s to %s'
#                      % source_frame, target_frame)
#     return transformation


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

def get_transform(msg):
    # br = tf2_ros.TransformBroadcaster()
    ts = TransformStamped()

    ts.header.stamp = msg[2]
    ts.header.frame_id = msg[4]
    ts.child_frame_id = msg[3]
    ts.transform.translation.x = msg[0][0]
    ts.transform.translation.y = msg[0][1]
    ts.transform.translation.z = msg[0][2]
    # q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.theta)
    ts.transform.rotation.x = msg[1][0]
    ts.transform.rotation.y = msg[1][1]
    ts.transform.rotation.z = msg[1][2]
    ts.transform.rotation.w = msg[1][3]
    return ts

# from geometry_msgs.msg import Quaternion, Vector3, Point
# import tf.transformations as t


def orientation_from_quaternion(q):
    return Quaternion(*q)


def orientation_from_euler(roll, pitch, yaw):
    q = t.quaternion_from_euler(roll, pitch, yaw)
    return orientation_from_quaternion(q)


def quaternion_from_orientation(o):
    return o.x, o.y, o.z, o.w


def euler_from_orientation(o):
    q = quaternion_from_orientation(o)
    return t.euler_from_quaternion(q)


def vector3_from_point(p):
    return Vector3(p.x, p.y, p.z)


def point_from_vector3(v):
    return Point(v.x, v.y, v.z)