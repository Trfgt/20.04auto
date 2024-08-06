#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
# import tf
import os
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from pyproj import Proj
from tracking_msg.msg import TrackingObjectArray
import pyproj
from math import pi
import math
from geometry_msgs.msg import Quaternion
from pyproj import CRS, Transformer
import numpy as np


class GPS2UTM:
    def __init__(self):
        rospy.loginfo("GPS2UTM is Created")
        # rospy.init_node('GPS2UTM', anonymous=True)
        rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        
        #----------------변수 초기화---------------------
        self.temp1=self.temp2= self.tempa=self.tempb=0
        #----------------------------------------------
        
     
    def gps_callback(self,_data):
        self.start_longitude = _data.longitude
        self.start_latitude = _data.latitude
        pass
    
    def imu_callback(self, _data):
        # rospy.loginfo("linear_acc %.2f",_data.linear_acceleration.y)
        self.Orientation = _data.orientation
        # rospy.loginfo("1, %.2f",self.y_vel)
        # rospy.loginfo(self.y_vel
        self.heading = self.calculate_heading_from_imu()
        # print(f"Heading: {self.heading}")
        pass
    
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
          
        # one object (test)
        # self.delta_x = _data.array[0].point.x
        # self.delta_y = _data.array[0].point.y
        
        # end_utm_easting, end_utm_northing = self.calculate_longitude_latitude()
        
        # a, b = self.calculate_longitude_latitude()
        # print("----end", end_utm_easting-self.temp1, end_utm_northing-self.temp2)
        # # end_utm_easting, end_utm_northing=a,b
        
        # self.temp1= end_utm_easting
        # self.temp2= end_utm_northing
        # print("End UTM Easting:",end_utm_easting, "End UTM Northing:", end_utm_northing)
        
        pass
        
        
    def calculate_longitude_latitude(self):
        # 시작 위치의 UTM 좌표 구하기
        transformer = Transformer.from_crs(CRS.from_epsg(4326), CRS.from_epsg(32652), always_xy=True)  # UTM Zone 52
        start_utm_easting, start_utm_northing = transformer.transform(self.start_longitude, self.start_latitude)
        print("Start UTM Easting:",start_utm_easting, "Start UTM Northing:", start_utm_northing)

        print("----start", start_utm_easting-self.tempa, start_utm_northing-self.tempb)
        
        self.tempa= start_utm_easting
        self.tempb= start_utm_northing
        
        
        
        # 헤딩값을 라디안 단위로 변환
        heading_rad = math.radians(self.heading)
        print("heading_rad", heading_rad)


        # 상대적인 x와 y 위치를 UTM 좌표계로 변환
        delta_utm_easting = self.delta_x * math.cos(heading_rad) - self.delta_y * math.sin(heading_rad)
        delta_utm_northing = self.delta_x * math.sin(heading_rad) + self.delta_y * math.cos(heading_rad)

        
        # 시작 위치에 UTM 좌표 변화량을 더하여 최종 UTM 좌표 구하기
        end_utm_easting = start_utm_easting + delta_utm_easting
        end_utm_northing = start_utm_northing + delta_utm_northing

        return end_utm_easting, end_utm_northing
    
    #------------------heading각 계산----------------------------

    def calculate_heading_from_imu(self):
        # Quaternion 메시지로부터 헤딩값 추출
        # quaternion 메시지의 x, y, z, w 값을 사용하여 헤딩값 계산
        x = self.Orientation .x
        y = self.Orientation .y
        z = self.Orientation .z
        w = self.Orientation .w

        # quaternion을 오일러 각도로 변환
        roll, pitch, yaw = self.quaternion_to_euler(x, y, z, w)

        # 헤딩값을 0~360 범위로 조정
        heading = (math.degrees(yaw) + 360) % 360

        return heading

    def quaternion_to_euler(self, x, y, z, w):
        # quaternion을 오일러 각도로 변환
        # 참고: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
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


    # # 헤딩값 계산
    # heading = calculate_heading_from_imu(orientation)
    # print(f"Heading: {heading}")

# # 상대적인 위치를 헤딩각에 따라 변환
# rotated_delta_x = delta_x * math.cos(math.radians(heading_offset)) - delta_y * math.sin(math.radians(heading_offset))
# rotated_delta_y = delta_x * math.sin(math.radians(heading_offset)) + delta_y * math.cos(math.radians(heading_offset))

# end_utm_easting, end_utm_northing = calculate_longitude_latitude(start_longitude, start_latitude, rotated_delta_x, rotated_delta_y, heading)
# print(f"End UTM Easting: {end_utm_easting}, End UTM Northing: {end_utm_northing}")




def run():
    rospy.init_node("gps2utm")
    new_classs= GPS2UTM()
    rospy.spin()
    

if __name__ == '__main__':
    run()