#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage
from pyproj import CRS, Transformer
from math import pi, radians, sin, cos
import math
import numpy as np
from tracking_msg.msg import TrackingObjectArray

    
class GPS2UTM:
    def __init__(self):
        rospy.loginfo("GPS2UTM is Created")
        rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        
        #----------------변수 초기화---------------------
        self.start_longitude = 0
        self.start_latitude = 0
        self.heading = 0
        self.delta_x = 0
        self.delta_y = 0
        #----------------------------------------------
        
    def gps_callback(self, _data):
        self.start_longitude = _data.longitude
        self.start_latitude = _data.latitude
    
    def imu_callback(self, _data):
        self.Orientation = _data.orientation
        self.heading = self.calculate_heading_from_imu()
    
    def lidar_callback(self, _data):
        total_obj_cnt = _data.size
        # UTM 좌표들을 저장할 빈 리스트 생성
        pointcloud = []

        for i in range(total_obj_cnt):
            self.delta_x = _data.array[i].point.x
            self.delta_y = _data.array[i].point.y
            end_utm_easting, end_utm_northing = self.calculate_longitude_latitude()
            # UTM 좌표를 리스트에 추가
            pointcloud.append([end_utm_easting, end_utm_northing])

        # 리스트를 NumPy 2차원 배열로 변환
        pointcloud_np = np.array(pointcloud)
        print(pointcloud_np)
        
    def calculate_longitude_latitude(self):
        # 시작 위치의 UTM 좌표 구하기
        transformer = Transformer.from_crs(CRS.from_epsg(4326), CRS.from_epsg(32652), always_xy=True)  # UTM Zone 52
        start_utm_easting, start_utm_northing = transformer.transform(self.start_longitude, self.start_latitude)

        # 헤딩값을 라디안 단위로 변환
        heading_rad = radians(self.heading)

        # 상대적인 x와 y 위치를 UTM 좌표계로 변환
        delta_utm_easting = self.delta_x * cos(heading_rad) - self.delta_y * sin(heading_rad)
        delta_utm_northing = self.delta_x * sin(heading_rad) + self.delta_y * cos(heading_rad)

        # 시작 위치에 UTM 좌표 변화량을 더하여 최종 UTM 좌표 구하기
        end_utm_easting = start_utm_easting + delta_utm_easting
        end_utm_northing = start_utm_northing + delta_utm_northing

        return end_utm_easting, end_utm_northing
    
    def calculate_heading_from_imu(self):
        # Quaternion 메시지로부터 헤딩값 추출
        x, y, z, w = self.Orientation.x, self.Orientation.y, self.Orientation.z, self.Orientation.w

        # quaternion을 오일러 각도로 변환
        roll, pitch, yaw = self.quaternion_to_euler(x, y, z, w)

        # 헤딩값을 0~360 범위로 조정
        heading = (math.degrees(yaw) + 360) % 360

        return heading

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        # quaternion을 오일러 각도로 변환
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw


def run():
    rospy.init_node("gps2utm")
    new_classs = GPS2UTM()
    rospy.spin()
    

if __name__ == '__main__':
    run()
