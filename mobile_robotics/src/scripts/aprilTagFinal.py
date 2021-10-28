#!/usr/bin/env python3

import rospy
import math
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Float64
from apriltag_ros.msg import AprilTagDetectionArray

stop_flag = 0
def get_init_position():
    data_odom = None
    while data_odom is None:
        try:
            data_odom = rospy.wait_for_message("/odom", Odometry, timeout=1)
        except:
            rospy.loginfo(
                "Current odom not ready yet, retrying for setting up init pose")

    current_pose = Point()
    current_pose.x = data_odom.pose.pose.position.x
    current_pose.y = data_odom.pose.pose.position.y
    current_pose.z = data_odom.pose.pose.position.z
    return current_pose


def callback(data):

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    speed = Twist()
    speed.linear.x = 0.1760

    t = 5
    stop_dist = trans[2] - 0.12
    while not rospy.is_shutdown():
        pub.publish(speed)
        current_pose = get_init_position()

        if current_pose.x > stop_dist:
            speed.linear.x = 0
            pub.publish(speed)
            rospy.spin()


if __name__ == '__main__':
    rospy.init_node('Node', anonymous=True)
    listener = tf.TransformListener()
    
    while(True):
        if(stop_flag == 0):
            try:
                (trans, rot) = listener.lookupTransform('/tag_0', '/base_footprint', rospy.Time(0))
                stop_flag = 1
            except (tf.LookupException, tf.ConnectivityException):
                continue
        sub = rospy.Subscriber(
            "/tag_detections", AprilTagDetectionArray, callback)
