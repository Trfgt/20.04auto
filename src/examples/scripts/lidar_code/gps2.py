#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
# import tf
import os
import rospkg
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
# from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Odometry
from pyproj import Proj
from tracking_msg.msg import TrackingObjectArray
# from tf.transformations import euler_from_quaternion,quaternion_from_euler
import pyproj
from math import pi
from std_msgs.msg import ColorRGBA
import math
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float32
from pyproj import CRS, Transformer
# from erp_driver.msg import erpCmdMsg
from visualization_msgs.msg import Marker, MarkerArray
from microstrain_inertial_msgs.msg import FilterHeading
import time
# from tf.transformations import euler_from_quaternion,quaternion_from_euler
# from custom_msg.msg import PointArray_msg.msg
from geometry_msgs.msg import Point
import numpy as np
    
class GPS2UTM:
    def __init__(self):
        rospy.loginfo("GPS2UTM is Created")
        # rospy.init_node('GPS2UTM', anonymous=True)
        # rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        # rospy.Subscriber("odom", Odometry, self.odom_callback)
        rospy.Subscriber("odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber("/nav/heading", FilterHeading, self.heading_callback)
        rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        #----------------변수 초기화---------------------
        self.temp1=self.temp2= self.tempa=self.tempb=0
        self.current_position = Point()
        self.is_odom=False
        #----------------------------------------------
        
    def create_cone_marker(self, position, orientation, color):
        cone_marker = Marker()
        cone_marker.header.frame_id = "velodyne"
        cone_marker.type = Marker.CYLINDER
        cone_marker.action = Marker.ADD
        cone_marker.pose.position.x = position[0]
        cone_marker.pose.position.y = position[1]
        cone_marker.pose.position.z = 0.0
        cone_marker.pose.orientation = orientation
        cone_marker.scale.x = 0.5
        cone_marker.scale.y = 0.5
        cone_marker.scale.z = 1.2
        cone_marker.color = color
        cone_marker.id = 0
        cone_marker.lifetime = rospy.Duration(1.0)
        return cone_marker  
    
    def create_cone_text(self, position, orientation, color):
        cone_marker = Marker()
        cone_marker.header.frame_id = "velodyne"
        cone_marker.type = Marker.TEXT_VIEW_FACING  # 텍스트 표시 설정
        cone_marker.text = f"({position[0]:.2f}, {position[1]:.2f})"  # 포지션 값을 문자열로 변환하여 설정
        cone_marker.pose.position.x = position[0]
        cone_marker.pose.position.y = position[1]
        cone_marker.pose.position.z = 0.0  # z 좌표에 높이를 설정할 수 있습니다.
        cone_marker.pose.orientation.w = 1.0  # Orientation을 설정합니다.
        cone_marker.scale.z = 0.2  # Text size
        cone_marker.color.a = 1.0  # Transparency
        cone_marker.color.r = 1.0  # Red
        cone_marker.color.g = 1.0  # Green
        cone_marker.color.b = 1.0  # Blue
        return cone_marker

    
    def calculate_bounding_box_center(self, bev_coords):
        center_x = (bev_coords[0] + bev_coords[2] + bev_coords[4] + bev_coords[6]) / 4
        center_y = (bev_coords[1] + bev_coords[3] + bev_coords[5] + bev_coords[7]) / 4
        return center_x, center_y
    
    def lidar_callback(self, _data):
        
        marker_array = MarkerArray()
        cone_positions = []
        cone_color = ColorRGBA()
        cone_color.a = 1.0
        cone_color.r = 1.0
        cone_color.g = 1.0
        cone_color.b = 0.0

        cone_orientation = Quaternion()
        cone_orientation.x = 0.0
        cone_orientation.y = 0.0
        cone_orientation.z = 0.0
        cone_orientation.w = 1.0

        for i, obj in enumerate(_data.array):
            bbox_center_x, bbox_center_y = self.calculate_bounding_box_center(obj.bev.data)
            self.delta_x = bbox_center_x
            self.delta_y = bbox_center_y
            end_utm_easting, end_utm_northing = self.calculate_longitude_latitude()
            print(end_utm_easting,end_utm_northing)
            position = (end_utm_easting, end_utm_northing)
            cone_positions.append(position)
            cone_marker = self.create_cone_marker(position, cone_orientation, cone_color)
            cone_color.b = 1.0 ### 
            cone_marker_text = self.create_cone_text(position,cone_orientation, cone_color)
            cone_marker.id = i
            marker_array.markers.append(cone_marker)
            marker_array.markers.append(cone_marker_text)
        
        total_obj_cnt = _data.size
        print("------------개수--------------")
        print("obs_cnt",total_obj_cnt)

        # UTM 좌표들을 저장할 빈 리스트 생성
        pointcloud = []
        if self.is_odom==True:
            print("easting: {:.2f}, northing: {:.2f}".format(self.current_position.x,self.current_position.y))
            for i in range(total_obj_cnt):
                self.delta_x = _data.array[i].point.x
                self.delta_y = _data.array[i].point.y
                end_utm_easting, end_utm_northing = self.calculate_longitude_latitude()
                
                # UTM 좌표를 리스트에 추가
                pointcloud.append([end_utm_easting, end_utm_northing])

            # 리스트를 NumPy 2차원 배열로 변환
            pointcloud_np = np.array(pointcloud)
            print("len",len(pointcloud_np))
            print("============좌표=============")
            print(pointcloud_np)
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
        
        
    def calculate_longitude_latitude(self):
     
        # 헤딩값을 라디안 단위로 변환
        heading_rad = self.vehicle_yaw
        # print("heading_rad", heading_rad)


        # 상대적인 x와 y 위치를 UTM 좌표계로 변환
        delta_utm_easting = self.delta_x * math.cos(heading_rad) - self.delta_y * math.sin(heading_rad)
        delta_utm_northing = self.delta_x * math.sin(heading_rad) + self.delta_y * math.cos(heading_rad)

        
        # 시작 위치에 UTM 좌표 변화량을 더하여 최종 UTM 좌표 구하기
        end_utm_easting = self.current_position.x + delta_utm_easting
        end_utm_northing = self.current_position.y + delta_utm_northing

        return end_utm_easting, end_utm_northing
   
    
    def heading_callback(self,msg):
        
        if self.is_odom==True:
            self.vehicle_yaw=msg.heading_rad

            self.vehicle_yaw=self.vehicle_yaw +self.init_heading

            if self.vehicle_yaw>np.pi:
                self.vehicle_yaw=-2*np.pi+self.vehicle_yaw
            elif self.vehicle_yaw<-np.pi:
                self.vehicle_yaw=2*np.pi+self.vehicle_yaw
    
    def odom_callback(self,msg):
        
        self.is_odom=True
        self.current_postion.x=msg.pose.pose.position.x
        self.current_postion.y=msg.pose.pose.position.y
        

def run():
    rospy.init_node("obs_gps")
    new_classs= GPS2UTM()
    rospy.spin()
    

if __name__ == '__main__':
    run()
