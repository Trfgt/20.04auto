#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker
from tracking_msg.msg import TrackingObjectArray
from geometry_msgs.msg import PoseArray, Pose
from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import Imu
import math

class LidarReceiver():
    def __init__(self):
        rospy.loginfo("LiDAR Receiver Object is Created")
        rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.bev_pub = rospy.Publisher('bev', PoseArray, queue_size=10)

    def calculate_bounding_box_center(self, bev_coords):
        center_x = (bev_coords[0] + bev_coords[2] + bev_coords[4] + bev_coords[6]) / 4
        center_y = (bev_coords[1] + bev_coords[3] + bev_coords[5] + bev_coords[7]) / 4
        return center_x, center_y

    def calculate_bounding_box_dimensions(self, bev_coords):
        width = math.sqrt((bev_coords[2] - bev_coords[0]) ** 2 + (bev_coords[3] - bev_coords[1]) ** 2)
        height = math.sqrt((bev_coords[4] - bev_coords[2]) ** 2 + (bev_coords[5] - bev_coords[3]) ** 2)
        return width, height

    def calculate_angle_with_vehicle(self, center_x, center_y):
        angle_rad = math.atan2(center_y, center_x)
        angle_deg = math.degrees(angle_rad)
        return angle_deg

    def calculate_distance_to_vehicle(self, center_x, center_y):
        distance = math.sqrt((center_x ) ** 2 + (center_y) ** 2)
        return distance
    
    def gps_callback(self,_data):
        self.start_longitude = _data.longitude
        self.start_latitude = _data.latitude
        pass

    def imu_callback(self, _data):
        self.Orientation = _data.orientation
        self.heading = self.calculate_heading_from_imu()
        pass

    def lidar_callback(self, msg):
        bev_msg = PoseArray()
        bev_msg.header = msg.header

        def get_vehicle_gps():
            vehicle_gps = (0.0, 0.0)  # 예시로 (0.0, 0.0)으로 설정
            return vehicle_gps

        for obj in msg.array:
            if len(obj.bev.data) < 8:
                rospy.logwarn("Invalid bounding box format: %s", obj.bev.data)
                continue

            bbox_center_x, bbox_center_y = self.calculate_bounding_box_center(obj.bev.data)
            bbox_width, bbox_height = self.calculate_bounding_box_dimensions(obj.bev.data)

            vehicle_x, vehicle_y = get_vehicle_gps()

            angle_deg = self.calculate_angle_with_vehicle(bbox_center_x, bbox_center_y)
            distance_to_vehicle = self.calculate_distance_to_vehicle(bbox_center_x, bbox_center_y)

            bev_pose = Pose()
            bev_pose.position.x = bbox_center_x
            bev_pose.position.y = bbox_center_y
            bev_pose.position.z = 0.0

            bev_pose.orientation.x = bbox_width
            bev_pose.orientation.y = bbox_height
            bev_pose.orientation.z = distance_to_vehicle
            bev_pose.orientation.w = angle_deg

            bev_msg.poses.append(bev_pose)

            rospy.loginfo("바운딩 박스 중심 좌표: ({}, {})".format(bbox_center_x, bbox_center_y))
            rospy.loginfo("바운딩 박스 크기: 가로={}, 세로={}".format(bbox_width, bbox_height))
            rospy.loginfo("차량과의 각도: {} 도".format(angle_deg))
            rospy.loginfo("차량과의 최단 거리: {} 미터".format(distance_to_vehicle))

        self.bev_pub.publish(bev_msg) 

def run():
    rospy.init_node("lidar_receiver")
    new_class = LidarReceiver()
    rospy.spin()

if __name__ == '__main__':
    run()