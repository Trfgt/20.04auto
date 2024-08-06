#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
# import tf
import os
# from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from pyproj import Proj
from tracking_msg.msg import TrackingObjectArray
import pyproj
from math import pi,sqrt
import math
from geometry_msgs.msg import Quaternion
from pyproj import CRS, Transformer
from custom_msg.msg import PointArray_msg # geometry_msgs/Point[] array -> custom msg 생성 
import numpy as np
import time
from nav_msgs.msg import Odometry


class GPS2UTM:
    def __init__(self):
        rospy.loginfo("GPS2UTM is Created")
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        # rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        self.pub=rospy.Publisher("/PointArray", PointArray_msg, queue_size=1)
        #----------------변수 초기화---------------------
        self.temp1=self.temp2= self.tempa=self.tempb=0
        self.wheelbase=1.1
        self.is_gps=False
        self.current_time=0
        self.is_odom=False
        self.x=None
        self.y=None
        self.heading_angle=None
        #----------------------------------------------
        
     
    def odom_callback(self,msg): # GPS pose, IMU heading angle data
        self.is_odom = True
        self.odom_status = msg
        # print(self.odom_status)

    def sub_odom_status(self,msg):
        self.heading_angle = msg.pose.pose.orientation.z
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y       

        return self.x, self.y, np.deg2rad(self.heading_angle)
    # utm 좌표계 및 heading angle 반환

    def imu_callback(self, _data):

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
        print(pointcloud)
        east_offset = 302459.942
        north_offset = 4122635.537
        print(total_obj_cnt)
        time_=time.time()

        # print(1/(time.time()-self.current_time))
        
        self.current_time=time.time()
        
        for i in range(total_obj_cnt):
            self.delta_x = _data.array[i].point.x + self.wheelbase
            self.delta_y = _data.array[i].point.y
            # self.delta는 라이다와 gps 위치 차이 고려
            # wheelbase는 약 1.1로 설정. 부착위치에 따라 변경 요구

            x=_data.array[i].point.x
            y=_data.array[i].point.y
            # 라이다 고유 데이터


            # print(x,y)
            obs_angle,obs_dist=self.cal_obs_data(x,y)
            # angle이랑 dist 계산
            # angle은 현재 degree 상태. -> cal_obs_data에 수정 가능.


            # obs_angle이 ego 기준으로 obs가 어디 각도에 위치해있는지
            # 코드 작성 방법
            # abs(angle)이 90도를 넘으면 y 기준으로 roi 안쪽에 위치해 있는지
            # 90도 이하면, obs_dist를 기준으로 안쪽에 위치해 있는지
            # 지피티로 검색해서 publish하는 것까지 작성.
            # break 사용 x 뒤에 부분까지 판단해야함.
            # 장애물이 ego와 충돌가능성에 대한 bool 대수로 반환.

            end_utm_easting, end_utm_northing = self.calculate_longitude_latitude()
            # print("obs_x: {:.6f}, obs_y: {:.6f}".format(end_utm_easting - east_offset,end_utm_northing - north_offset))
             # UTM 좌표를 Point 메시지로 변환하여 리스트에 추가
            
            if obs_dist <= 5:
                print("obs_x: {:.6f}, obs_y: {:.6f}".format(end_utm_easting - east_offset,end_utm_northing - north_offset))
                print("obs_angle: {:.3f}, obs_dist: {:.3f}".format(obs_angle,obs_dist))
                point_msg = Point()
                point_msg.x = end_utm_easting - east_offset
                point_msg.y = end_utm_northing - north_offset
                point_msg.z = obs_angle  # Assuming z coordinate is 0 for simplicity
                pointcloud.append(point_msg)
            # dist <= 5이하일 때만 장애물로 추가.
            # 헤딩값 계산
        heading = self.calculate_heading_from_imu()
        print(f"Heading: {heading}")

        # custom_msg/PointArray_msg.msg 메시지 생성
        msg = PointArray_msg()
        print(1/(time.time()-time_))
        # geometry_msgs/Point[] array 필드에 그대로 할당
        msg.array = pointcloud
        self.pub.publish(msg)

            # # UTM 좌표를 리스트에 추가
            # pointcloud.append([end_utm_easting, end_utm_northing])

        # 리스트를 NumPy 2차원 배열로 변환
        # pointcloud_np = np.array(pointcloud)
        # print(pointcloud_np)
          
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
        
    def cal_obs_data(self,delta_x,delta_y):
        x=delta_x
        y=delta_y

        obs_angle = np.rad2deg(math.atan2(y,x))
        obs_dist = np.sqrt(x**2+y**2)

        return obs_angle, obs_dist
        
    def calculate_longitude_latitude(self):

        start_utm_easting, start_utm_northing, ego_head = self.sub_odom_status(self.odom_status)
        
        # 차량 UTM 좌표
        # print("Start UTM Easting:",start_utm_easting, "Start UTM Northing:", start_utm_northing)

        # 프레임별 차량 UTM좌표 차이 
        # print("----start", start_utm_easting-self.tempa, start_utm_northing-self.tempb)
        
    
        # 헤딩값을 라디안 단위로 변환
        heading_rad = (ego_head+2*pi)%(2*pi)
        # 이미 radian 단위
        # print("heading_rad", heading_rad*180/pi)

        self.tempa= start_utm_easting
        self.tempb= start_utm_northing


        # 상대적인 x와 y 위치를 UTM 좌표계로 변환
        delta_utm_easting = self.delta_x * math.cos(heading_rad) - self.delta_y * math.sin(heading_rad)
        delta_utm_northing = self.delta_x * math.sin(heading_rad) + self.delta_y * math.cos(heading_rad)
        # print(he/ading_rad)
        # obs_angle=(math.atan2(delta_utm_northing,delta_utm_easting)-heading_rad+2*pi)%(2*pi)
        # dist=sqrt(delta_utm_easting**2+delta_utm_northing**2)
        # # print('dist:',dist)
        # if obs_angle > pi:
        #     obs_angle-=2*pi
        # if abs(obs_angle)*180/pi<20 and dist < 5:
        #     print('dist:',dist)
        # print("obs_angle",obs_angle*180/pi)
        # 부호가 반대일 수도 있음.
        # 차량의 방향을 기준으로 장애물이 어디에 위치돼 있는지
        
        
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