#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import rospy
import tf
from math import pi
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped


class Ego_listener():
    def __init__(self):
        rospy.init_node('status_listener', anonymous=True)
        rospy.Subscriber("odom_gps", Odometry, self.odom_callback)
        # Static Transform 생성
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.send_static_transform()
        rospy.spin()

    # Static Transform 전송 (Ego와 velodyne 간의 변환)
    def send_static_transform(self):
        static_transformStamped = TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "Ego"
        static_transformStamped.child_frame_id = "velodyne"

        static_transformStamped.transform.translation.x = 1.44
        static_transformStamped.transform.translation.y = 0
        static_transformStamped.transform.translation.z = 0

        quat = tf.transformations.quaternion_from_euler(0, 0, 0) # yaw, pitch, roll
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        self.static_broadcaster.sendTransform(static_transformStamped)

    def odom_callback(self,msg):
        self.is_odom = True

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = msg.pose.pose.position.z

        q =tf.transformations.quaternion_from_euler(0., 0.,self.yaw)
        # self.orientation_x = msg.pose.pose.orientation.x
        # self.orientation_y = msg.pose.pose.orientation.y
        # self.orientation_z = msg.pose.pose.orientation.z
        # self.orientation_w = msg.pose.pose.orientation.w

        self.orientation_x = q[0]
        self.orientation_y = q[1]
        self.orientation_z = q[2]
        self.orientation_w = q[3]

        br = tf.TransformBroadcaster()
        br.sendTransform((self.x, self.y, 1),
                        (self.orientation_x, self.orientation_y, self.orientation_z, self.orientation_w),
                        rospy.Time.now(),
                        "Ego",
                        "map")
                        

if __name__ == '__main__':
    try:
        tl=Ego_listener()
    except rospy.ROSInternalException:
        pass
