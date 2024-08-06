#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from tracking_msg.msg import TrackingObjectArray

class LidarReceiver():
    def __init__(self):
        rospy.loginfo("LiDAR Receiver Object is Created")
        rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        self.marker_pub = rospy.Publisher('bounding_boxes_markers', Marker, queue_size=10)

    def lidar_callback(self, msg):
        marker_array = Marker()
        marker_array.header = msg.header
        marker_array.type = Marker.CUBE_LIST
        marker_array.action = Marker.ADD
        marker_array.ns = "bounding_boxes"
        marker_array.id = 0
        marker_array.scale.x = 1.0  # 바운딩 박스 두께 설정
        marker_array.color.r = 1.0
        marker_array.color.g = 0.0
        marker_array.color.b = 0.0
        marker_array.color.a = 1.0

        for obj in msg.array:
            # if len(obj.bbox2d) < 4:  # 요소의 개수가 4개보다 적은 경우 처리
            #     rospy.logwarn("Invalid bbox2d format: %s", obj.bbox2d)
            #     continue

            # x1, y1, x2, y2 = obj.bbox2d[0], obj.bbox2d[1], obj.bbox2d[2], obj.bbox2d[3]
            rospy.loginfo(obj.bev.data)
            rospy.loginfo(obj.bbox)


            # bbox_center_x = (x1 + x2) / 2
            # bbox_center_y = (y1 + y2) / 2

        #     bbox_width = abs(x2 - x1)
        #     bbox_height = abs(y2 - y1)

        #     marker = Marker()
        #     marker.header = msg.header
        #     marker.type = Marker.CUBE
        #     marker.action = Marker.ADD
        #     marker.ns = "bounding_boxes"
        #     marker.id = len(marker_array.points)
        #     marker.pose.position.x = bbox_center_x
        #     marker.pose.position.y = bbox_center_y
        #     marker.pose.position.z = 0.0
        #     marker.scale.x = bbox_width
        #     marker.scale.y = bbox_height
        #     marker.scale.z = 0.1  # 바운딩 박스 높이 설정
        #     marker.color.r = 1.0
        #     marker.color.g = 0.0
        #     marker.color.b = 0.0
        #     marker.color.a = 0.5  # 바운딩 박스 투명도 설정

        #     marker_array.points.append(marker.pose.position)

        # self.marker_pub.publish(marker_array)

def run():
    rospy.init_node("lidar_receiver")
    new_class = LidarReceiver()
    rospy.spin()

if __name__ == '__main__':
    run()
