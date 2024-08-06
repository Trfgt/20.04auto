#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from tracking_msg.msg import TrackingObjectArray
import math

class LidarReceiver():
    def __init__(self):
        rospy.loginfo("LiDAR Receiver Object is Created")
        rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        self.marker_pub = rospy.Publisher('bounding_boxes_markers', Marker, queue_size=10)

    def calculate_bounding_box_center(self, bev_coords):
        # bev_coords: [x1, y1, x2, y2, x3, y3, x4, y4]
        center_x = (bev_coords[0] + bev_coords[2] + bev_coords[4] + bev_coords[6]) / 4
        center_y = (bev_coords[1] + bev_coords[3] + bev_coords[5] + bev_coords[7]) / 4
        return center_x, center_y

    def calculate_bounding_box_dimensions(self, bev_coords):
        # bev_coords: [x1, y1, x2, y2, x3, y3, x4, y4]
        width = math.sqrt((bev_coords[2] - bev_coords[0]) ** 2 + (bev_coords[3] - bev_coords[1]) ** 2)
        height = math.sqrt((bev_coords[4] - bev_coords[2]) ** 2 + (bev_coords[5] - bev_coords[3]) ** 2)
        return width, height

    def calculate_angle_with_vehicle(self, center_x, center_y, vehicle_x, vehicle_y):
        angle_rad = math.atan2(center_y - vehicle_y, center_x - vehicle_x)
        angle_deg = math.degrees(angle_rad)
        return angle_deg

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

        # Morai에서 GPS 정보를 받아오는 부분은 Morai 라이브러리와 ROS의 메시지 송수신 방법을 따릅니다.
        # 아래의 코드는 Morai에서 GPS 정보를 받아오는 예시이며, Morai 라이브러리에 따라 구현 방법이 다를 수 있습니다.
        # 아래의 예시 코드는 차량의 GPS 위치를 받아오는 함수가 Morai 라이브러리에 있다고 가정합니다.
        def get_vehicle_gps():
            # Morai 라이브러리에서 차량의 GPS 위치를 받아오는 함수
            vehicle_gps = (0.0, 0.0)  # 예시로 (0.0, 0.0)으로 설정
            return vehicle_gps

        for obj in msg.array:
            if len(obj.bev.data) < 8:
                rospy.logwarn("Invalid bounding box format: %s", obj.bev.data)
                continue

            bbox_center_x, bbox_center_y = self.calculate_bounding_box_center(obj.bev.data)
            bbox_width, bbox_height = self.calculate_bounding_box_dimensions(obj.bev.data)

            # Morai 라이브러리로부터 차량의 GPS 위치 정보를 받아옵니다.
            vehicle_x, vehicle_y = get_vehicle_gps()

            angle_deg = self.calculate_angle_with_vehicle(bbox_center_x, bbox_center_y, vehicle_x, vehicle_y)

            # TODO: 바운딩 박스의 가로 세로 길이와 차량과의 각도를 활용한 추가 로직을 작성하세요.
            # 예를 들어, 계산한 길이와 각도를 이용해 바운딩 박스를 그리거나, 다른 연산을 수행할 수 있습니다.

            rospy.loginfo("바운딩 박스 중심 좌표: ({}, {})".format(bbox_center_x, bbox_center_y))
            rospy.loginfo("바운딩 박스 크기: 가로={}, 세로={}".format(bbox_width, bbox_height))
            rospy.loginfo("차량과의 각도: {} 도".format(angle_deg))

def run():
    rospy.init_node("lidar_receiver")
    new_class = LidarReceiver()
    rospy.spin()

if __name__ == '__main__':
    run()
