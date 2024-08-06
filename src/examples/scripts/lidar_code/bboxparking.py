#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from tracking_msg.msg import TrackingObjectArray

class LidarReceiver():
    def __init__(self):
        rospy.loginfo("LiDAR Receiver Object is Created")
        rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        self.marker_pub = rospy.Publisher('bounding_boxes_markers', Marker, queue_size=10)

        # 주차장의 좌표를 정의
        self.parking_lots = [
            (19.44, 1104.7), (23.55, 1102.27), (22.41, 1100.31), (18.42, 1102.55),
            (18.42, 1102.55), (22.41, 1100.31), (21.21, 1098.14), (17.12, 1100.4),
            (17.12, 1100.4), (21.21, 1098.14), (19.96, 1095.76), (16, 1098.23),
            (16, 1098.23), (19.96, 1095.76), (18.82, 1093.87), (14.67, 1095.94)
        ]

        # Scale factor to adjust coordinates for better visualization
        self.scale_factor = 1
        self.offset_x = 0
        self.offset_y = 0

    def calculate_bounding_box_center(self, bev_coords):
        center_x = (bev_coords[0] + bev_coords[2] + bev_coords[4] + bev_coords[6]) / 4
        center_y = (bev_coords[1] + bev_coords[3] + bev_coords[5] + bev_coords[7]) / 4
        return center_x, center_y

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
            if len(obj.bev.data) < 8:
                rospy.logwarn("Invalid bounding box format: %s", obj.bev.data)
                continue

            bbox_center_x, bbox_center_y = self.calculate_bounding_box_center(obj.bev.data)

            rospy.loginfo("Obstacle Coordinates: ({}, {})".format(bbox_center_x, bbox_center_y))

            marker = Marker()
            marker.header = msg.header
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.id = obj.id

            for i in range(4):
                p = Point()
                p.x = obj.bev.data[i * 2] * self.scale_factor - self.offset_x
                p.y = obj.bev.data[i * 2 + 1] * self.scale_factor - self.offset_y
                p.z = 0.0
                marker.points.append(p)

            p = Point()
            p.x = obj.bev.data[0] * self.scale_factor - self.offset_x
            p.y = obj.bev.data[1] * self.scale_factor - self.offset_y
            p.z = 0.0
            marker.points.append(p)

            self.marker_pub.publish(marker)

            # 주차장 좌표를 이어서 사각형으로 시각화
            for i in range(0, len(self.parking_lots), 4):
                rect_marker = Marker()
                rect_marker.header = msg.header
                rect_marker.type = Marker.LINE_STRIP
                rect_marker.action = Marker.ADD
                rect_marker.scale.x = 0.05
                rect_marker.color.a = 1.0
                rect_marker.color.r = 0.0
                rect_marker.color.g = 1.0
                rect_marker.color.b = 0.0
                rect_marker.id = i

                for j in range(4):
                    p = Point()
                    p.x = self.parking_lots[i + j][0] - self.offset_x
                    p.y = self.parking_lots[i + j][1] - self.offset_y
                    p.z = 0.0
                    rect_marker.points.append(p)

                p = Point()
                p.x = self.parking_lots[i][0] - self.offset_x
                p.y = self.parking_lots[i][1] - self.offset_y
                p.z = 0.0
                rect_marker.points.append(p)

                self.marker_pub.publish(rect_marker)

def run():
    rospy.init_node("lidar_receiver")
    new_class = LidarReceiver()
    rospy.spin()

if __name__ == '__main__':
    run()
