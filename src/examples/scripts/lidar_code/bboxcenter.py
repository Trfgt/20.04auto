#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from tracking_msg.msg import TrackingObjectArray
from geometry_msg.msg import Pose

class LidarReceiver():
    def __init__(self):
        rospy.loginfo("LiDAR Receiver Object is Created")
        rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        
        self.marker_pub = rospy.Publisher('bounding_boxes_markers', Marker, queue_size=10)
        self.bbox_pub = rospy.Publisher('bounding_boxes', Pose, queue_size=10)

    def calculate_bounding_box_center(self, bev_coords):
        # bev_coords: [x1, y1, x2, y2, x3, y3, x4, y4]
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
        marker_array.scale.x = 1.0  # 바운딩 박스 두께 설정
        marker_array.color.r = 1.0
        marker_array.color.g = 0.0
        marker_array.color.b = 0.0
        marker_array.color.a = 1.0

        for obj in msg.array:
            if len(obj.bev.data) < 8:  # 요소의 개수가 8개보다 적은 경우 처리
                rospy.logwarn("Invalid bounding box format: %s", obj.bev.data)
                continue

            bbox_center_x, bbox_center_y = self.calculate_bounding_box_center(obj.bev.data)

            # TODO: 바운딩 박스의 중심좌표를 활용한 추가 로직을 작성하세요.
            # 예를 들어, 중심좌표를 이용해 바운딩 박스를 그리거나, 다른 연산을 수행할 수 있습니다.

            rospy.loginfo("Bounding Box Center Coordinates: ({}, {})".format(bbox_center_x, bbox_center_y))

def run():
    rospy.init_node("lidar_receiver")
    new_class = LidarReceiver()
    rospy.spin()

if __name__ == '__main__':
    run()