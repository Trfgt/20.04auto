#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from tracking_lib.msg import TrackingObjectArray
from morai_msgs.msg import EgoVehicleStatus

class LidarReceiver():
    def __init__(self):
        rospy.loginfo("LiDAR Receiver Object is Created")
        rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.Ego_callback)

    def lidar_callback(self, msg):
        marker_array = Marker()
        marker_array.header = msg.header
        marker_array.type = Marker.CUBE_LIST
        marker_array.action = Marker.ADD
        marker_array.ns = "bounding_boxes"
        marker_array.id = 0
        marker_array.scale.x = 1.0
        marker_array.color.r = 1.0
        marker_array.color.g = 0.0
        marker_array.color.b = 0.0
        marker_array.color.a = 1.0

        for obj in msg.array:
            bbox2d = obj.bbox2d.data
            rospy.loginfo("bbox2d: %s", bbox2d)  # 디버깅 문장 추가

            if len(bbox2d) != 4:  # 요소의 개수가 4개가 아닌 경우 처리
                rospy.logwarn("Invalid bbox2d format: %s", bbox2d)
                continue

            x1, y1, x2, y2 = bbox2d[0], bbox2d[1], bbox2d[2], bbox2d[3]

            bbox_center_x = (x1 + x2) / 2
            bbox_center_y = (y1 + y2) / 2

            bbox_width = abs(x2 - x1)
            bbox_height = abs(y2 - y1)

            marker = Marker()
            marker.header = msg.header
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.ns = "bounding_boxes"
            marker.id = len(marker_array.points)
            marker.pose.position.x = bbox_center_x
            marker.pose.position.y = bbox_center_y
            marker.pose.position.z = 0.0
            marker.scale.x = bbox_width
            marker.scale.y = bbox_height
            marker.scale.z = 0.1  # 바운딩 박스 높이 설정
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5  # 바운딩 박스 투명도 설정

            marker_array.points.append(marker.pose.position)

            marker_pub.publish(marker_array)  # marker_pub은 이곳에서 사용해야 함

    def Ego_callback(self, _data):
        self.car_vel_x = _data.velocity.x
        self.car_vel_y = _data.velocity.y


def run():
    rospy.init_node("lidar")
    new_class = LidarReceiver()
    rospy.spin()

if __name__ == '__main__':
    run()
