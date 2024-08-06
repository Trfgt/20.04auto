#!/usr/bin/env python2
# -*- coding: utf-8 -*-
 
import rospy
import tf
import os
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from pyproj import Proj
from pyproj import CRS, Transformer
from math import pi
# from morai_msgs.msg import CtrlCmd,EgoVehicleStatus

# gpsimu_parser 는 GPS, IMU 센서 데이터를 받아 차량의 상대위치를 추정하는 예제입니다.

# 노드 실행 순서 
# 1. 변환 하고자 하는 좌표계를 선언
# 2. 송신 될 Odometry 메세지 변수 생성
# 3. 위도 경도 데이터 UTM 죄표로 변환
# 4. Odometry 메세지 변수에 차량의 위치 및 상태 데이터 담기
# 5. Odometry 메세지 Publish

class GPSIMUParser:
    def __init__(self):
        rospy.init_node('GPS_IMU_parser', anonymous=True)
        self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        # rospy.Subscriber("/Ego_topic",EgoVehicleStatus, self.status_callback)
        self.odom_pub = rospy.Publisher('/odom',Odometry, queue_size=1)
        # 초기화
        self.x, self.y = None, None
        self.is_imu=False
        self.is_gps=False
        self.status_msg=None

        #TODO: (1) 변환 하고자 하는 좌표계를 선언
        self.proj_UTM = Proj(proj='utm',zone=52, ellps='WGS84', preserve_units=False)

        #TODO: (2) 송신 될 Odometry 메세지 변수 생성
        self.odom_msg=Odometry()
        self.odom_msg.header.frame_id='/odom'
        self.odom_msg.child_frame_id='/base_link'

        rate = rospy.Rate(30) # 30hz
        while not rospy.is_shutdown():
            if self.is_imu==True and self.is_gps == True:
                self.convertLL2UTM()

                #TODO: (5) Odometry 메세지 Publish
                self.odom_pub.publish(self.odom_msg)

                os.system('clear')
                print(self.odom_msg)
                # print("lat:{:.2f}, lon:{:.2f}".format(self.lat,self.lon))
                # print("ego_x:{:.2f}, ego_y:{:.2f}".format(self.status_msg.position.x,self.status_msg.position.y))

                rate.sleep()

    def navsat_callback(self, gps_msg):

        self.lat = gps_msg.latitude
        self.lon = gps_msg.longitude
        self.e_o = gps_msg.eastOffset
        self.n_o = gps_msg.northOffset

        self.is_gps=True

    #TODO: (3) 위도 경도 데이터 UTM 죄표로 변환
    def convertLL2UTM(self):    
        #xy_zone = self.proj_UTM(self.lon, self.lat)
        transformer = Transformer.from_crs(CRS.from_epsg(4326), CRS.from_epsg(32652), always_xy=True)
        self.x, self.y = transformer.transform(self.lon, self.lat)

        #self.x = xy_zone[0] - self.e_o
        #self.y = xy_zone[1] - self.n_o
        east_offset = 302459.942
        north_offset = 4122635.537
        #TODO: (4) Odometry 메세지 변수에 차량의 위치 및 상태 데이터 담기
        self.odom_msg.header.stamp = rospy.get_rostime()
        self.odom_msg.pose.pose.position.x = self.x - east_offset
        self.odom_msg.pose.pose.position.y = self.y - north_offset
        self.odom_msg.pose.pose.position.z = 0.

    def imu_callback(self, data):

        #TODO: (4) Odometry 메세지 변수에 차량의 위치 및 상태 데이터 담기
        self.odom_msg.pose.pose.orientation.x = data.orientation.x
        self.odom_msg.pose.pose.orientation.y = data.orientation.y
        self.odom_msg.pose.pose.orientation.z = data.orientation.z
        self.odom_msg.pose.pose.orientation.w = data.orientation.w

        self.is_imu=True

    # def status_callback(self,msg): ## Vehicl Status Subscriber 
    #     self.is_status=True
    #     self.status_msg=msg

if __name__ == '__main__':
    try:
        GPS_IMU_parser = GPSIMUParser()
    except rospy.ROSInterruptException:
        pass
