#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
# import tf
import os
import rospkg
from std_msgs.msg import Float32MultiArray,Bool,Int32
from sensor_msgs.msg import Imu
# from morai_msgs.msg import GPSMessage
from geometry_msgs.msg import Point
from pyproj import Proj
from tracking_msg.msg import TrackingObjectArray
import pyproj
from math import pi,sqrt
import math
from geometry_msgs.msg import Quaternion
from pyproj import CRS, Transformer
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from custom_msg.msg import PointArray_msg # geometry_msgs/Point[] array -> custom msg 생성 
import numpy as np
import time
from nav_msgs.msg import Odometry
from microstrain_inertial_msgs.msg import FilterHeading

#############
#### 본선 ####
#############

class GPS2UTM:
    def __init__(self):
        rospy.loginfo("GPS2UTM is Created")
        rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        # rospy.Subscriber("/nav/heading", FilterHeading, self.heading_callback)
        rospy.Subscriber("odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber("utm_cal_trigger", Bool, self.delivery_trigger_callback)
        rospy.Subscriber("/traffic_sign",Int32,self.vision_callback)
        self.marker_pub = rospy.Publisher('markers', MarkerArray, queue_size=10)  # Define the marker publisher
        self.bev_pub = rospy.Publisher('bev', PoseArray, queue_size=10)
        self.delivery_utm_pub=rospy.Publisher('utm_sign', Float32MultiArray, queue_size=10)
        self.delivery_stop_pub=rospy.Publisher('/observed_sign', Bool, queue_size=10)
        
        #----------------변수 초기화---------------------
        self.temp1=self.temp2= self.tempa=self.tempb=0
        self.wheelbase=1.44
        self.is_gps=False
        self.current_time=0
        #----------------------------------------------       
        self.vehicle_yaw=0.
        self.is_odom=False
        self.current_position=Point()
        self.marker_array = MarkerArray()

        rate = rospy.Rate(20) # 20hz
        self.rospack = rospkg.RosPack()

    
    def delivery_trigger_callback(self, delivery):
        self.delivery_flag = delivery
    
    def vision_callback(self,vision_sign):
        self.vision_num = vision_sign

    def lidar_callback(self, _data):
        Int32
        total_obj_cnt = _data.size
        

        # UTM 좌표들을 저장할 빈 리스트 생성
        pointcloud = []
        delivery_signs=[]
        east_offset = 0.
        north_offset = 0.
        # print("=================")
        # print("total_cnt")
        # print(total_obj_cnt)
        # print("=================")
        # print("rate")
        # print(1/(time.time()-self.current_time))
        delivery_sign_cnt=0
        self.current_time=time.time()

        bev_msg = PoseArray()
        bev_msg.header = _data.header
        
        for obj in _data.array:

            if len(obj.bev.data) < 8:
                rospy.logwarn("Invalid bounding box format: %s", obj.bev.data)
                continue
            # exception

            bbox_center_x, bbox_center_y = self.calculate_bounding_box_center(obj.bev.data)
            bbox_width, bbox_height = self.calculate_bounding_box_dimensions(obj.bev.data)

            self.delta_x = bbox_center_x + self.wheelbase
            self.delta_y = bbox_center_y
            end_utm_easting, end_utm_northing = self.calculate_longitude_latitude()
            # self.delta는 라이다와 gps 위치 차이 고려
            # wheelbase는 약 1.1로 설정. 부착위치에 따라 변경 요구

            x=bbox_center_x 
            y=bbox_center_y
            # 라이다 고유 데이터

            obs_angle,obs_dist=self.cal_obs_data(x,y)
            # angle이랑 dist 계산
            # angle은 현재 degree 상태. -> cal_obs_data에 수정 가능.

            # if(0<x<15 and -4<y<0 and bbox_height>0.5 and 0.45>bbox_width>0.03):
            if(2<x<15 and -4<y<0 and 0.7>bbox_height>0.3):
            # if(1<x<18 and -3.7<y<0 ): # parking utm coordinate check
                print("x,y",x,y,"W",bbox_width,"H",bbox_height)
                print("obs_x: {:.6f}, obs_y: {:.6f}".format(end_utm_easting - east_offset,end_utm_northing - north_offset))
                delivery_signs.append((x,y,end_utm_easting,end_utm_northing))
                delivery_sign_cnt+=1
                # marker = Marker()
                # marker.header = _data.header
                # marker.header.frame_id = "map"
                # marker.type = Marker.SPHERE  # You can choose the marker type you prefer
                # marker.action = Marker.ADD
                # marker.pose.position.x = end_utm_easting
                # marker.pose.position.y = end_utm_northing
                # marker.pose.position.z = 1.0
                # marker.scale.x = 1.0  # Adjust the marker size as needed
                # marker.scale.y = 1.0
                # marker.scale.z = 1.0
                # marker.color.a = 1.0  # Alpha (transparency)
                # marker.color.r = 1.0  # Red color
                # marker.color.g = 0.0  # Green color
                # marker.color.b = 0.0  # Blue color
                # marker.id=delivery_sign_cnt
                # self.marker_array.markers.append(marker)

                marker1 = Marker()
                marker1.header = _data.header
                marker1.header.frame_id = "velodyne"
                marker1.type = Marker.SPHERE  # You can choose the marker type you prefer
                marker1.action = Marker.ADD
                marker1.pose.position.x = bbox_center_x
                marker1.pose.position.y = bbox_center_y
                marker1.pose.position.z = 1.0
                marker1.scale.x = 1.0  # Adjust the marker size as needed
                marker1.scale.y = 1.0
                marker1.scale.z = 1.0
                marker1.color.a = 1.0  # Alpha (transparency)
                marker1.color.r = 0.0  # Red color
                marker1.color.g = 0.0  # Green color
                marker1.color.b = 1.0  # Blue color
                marker1.id=delivery_sign_cnt+10
                self.marker_array.markers.append(marker1)

            
            # print("obs_x: {:.6f}, obs_y: {:.6f}".format(end_utm_easting - east_offset,end_utm_northing - north_offset))
             # UTM 좌표를 Point 메시지로 변환하여 리스트에 추가
            
            if obs_dist <= 10:
                if bbox_center_x>0.3:
                    # print("============obs_data==================")
                    # print("obs_x: {:.6f}, obs_y: {:.6f}".format(end_utm_easting - east_offset,end_utm_northing - north_offset))
                    # print("obs_angle: {:.3f}, obs_dist: {:.3f}".format(obs_angle,obs_dist))
                    # print("obs_width: {:.3f}, obs_height: {:.3f}".format(bbox_width,bbox_height))

                    bev_pose = Pose()
                    bev_pose.position.x = end_utm_easting
                    bev_pose.position.y = end_utm_northing
                    bev_pose.position.z = 0.0

                    bev_pose.orientation.x = bbox_width
                    bev_pose.orientation.y = bbox_height
                    bev_pose.orientation.z = obs_dist
                    bev_pose.orientation.w = obs_angle

                    bev_msg.poses.append(bev_pose)
            # dist <= 5이하일 때만 장애물로 추가.
        
        print("**************")
        print("cnt",delivery_sign_cnt)
        print("**************")
        Delivery_target_idx = None 
        # Delivery_stop_point
        msg=Bool()
        # self.vision_num=12
        if(delivery_sign_cnt>=3 and self.vision_num==2):#true wlthr
            msg=True
            print("Delivery")
            print("Delivery")
            Delivery_target_idx=1
            delivery_signs.sort(key=lambda x: x[0])  # x
        else:
            msg=False

        print("self.vision_num",self.vision_num)
        self.delivery_stop_pub.publish(msg)

        #### Delivery: topic from vision ####
        # A1:11, A2:12, A3:13 , idx = B1:0, B2:1, B3:2
        # Delivery_target_idx = {11:0, 12:1, 13:2}.get(self.vision_num,None)
        # self.vision_num=12
        
        # Delivery_target_idx = {11:0, 12:1, 13:2}.get(int(self.vision_num),None)
        # Delivery_target_idx=1
        # print(delivery_signs)
        
        #####delivery utm coordinate calculate#####
        # if self.is_odom and self.delivery_flag==True: # planner
        if self.is_odom and Delivery_target_idx is not None and len(delivery_signs)>3 and self.delivery_flag==True:
                
                print("DELIVERY target-idx",Delivery_target_idx)

                position_x = self.current_position.x
                position_y = self.current_position.y

                pkg_name = 'examples' 

                path_name = "ajou_univ_final"
                pkg_path = self.rospack.get_path(pkg_name)
                full_path = pkg_path + '/path' + '/' + path_name+'.txt'
                f = open(full_path,'r')    
                lines = f.readlines()

                delivery_line_x = []
                delivery_line_y = []

                for line in lines :
                    tmp = line.split()
                    delivery_line_x.append(float(tmp[0]))
                    delivery_line_y.append(float(tmp[1]))
                f.close()
                
                self.delivery_zone_x = delivery_signs[Delivery_target_idx][2]
                self.delivery_zone_y = delivery_signs[Delivery_target_idx][3]
                # self.delivery_zone_x = delivery_signs[0][2]
                # self.delivery_zone_y = delivery_signs[0][3]

                target_utm = self.generate_delivery_path(position_x, position_y, delivery_line_x, delivery_line_y, self.delivery_zone_x, self.delivery_zone_y)
                print("target_utm",target_utm)
                target_utm_msg = Float32MultiArray()
                target_utm_msg.data = [target_utm[0],target_utm[1]]  # You can put any other 32-bit float numbers here   
                self.delivery_utm_pub.publish(target_utm_msg) 

                #### Delivery target_utm check marker ####
                # marker1 = Marker()
                # marker1.header = _data.header
                # marker1.header.frame_id = "map"
                # marker1.type = Marker.SPHERE  # You can choose the marker type you prefer
                # marker1.action = Marker.ADD
                # marker1.pose.position.x = target_utm[0]
                # marker1.pose.position.y = target_utm[1]
                # marker1.pose.position.z = 1.0
                # marker1.scale.x = 1.0  # Adjust the marker size as needed
                # marker1.scale.y = 1.0
                # marker1.scale.z = 1.0
                # marker1.color.a = 1.0  # Alpha (transparency)
                # marker1.color.r = 0.0  # Red color
                # marker1.color.g = 0.0  # Green color
                # marker1.color.b = 1.0  # Blue color
                # marker1.id=1000
                # self.marker_array.markers.append(marker1)                  
        else:
            print("Not calculate delivery")

        # custom_msg/PointArray_msg.msg 메시지 생성
        self.bev_pub.publish(bev_msg)
        self.marker_pub.publish(self.marker_array)
    

        pass

    def generate_delivery_path(self, x, y, delivery_line_x, delivery_line_y, delivery_zone_x, delivery_zone_y):

        distance_list = []

        for i in range(len(delivery_line_x)):
            distance = sqrt(pow(delivery_line_x[i] - delivery_zone_x,2) + pow(delivery_line_y[i] - delivery_zone_y,2))
            distance_list.append(distance)

        minimum_distance = min(distance_list)
        minimum_distance_index = distance_list.index(minimum_distance)

        delivery_path_x = []
        delivery_path_y = []

        for i in range(minimum_distance_index - 15, minimum_distance_index):
            delivery_path_x.append(delivery_line_x[i])
            delivery_path_y.append(delivery_line_y[i])
        
        target_utm=(delivery_path_x[minimum_distance_index],delivery_path_y[minimum_distance_index])

        return target_utm

    def odom_callback(self,msg):
        
        self.is_odom=True
        
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y
        self.vehicle_yaw=msg.pose.pose.position.z
        
    def cal_obs_data(self,delta_x,delta_y):
        x=delta_x
        y=delta_y

        obs_angle = np.rad2deg(math.atan2(y,x))
        obs_dist = np.sqrt(x**2+y**2)

        return obs_angle, obs_dist
        
    def calculate_longitude_latitude(self):
        # 시작 위치의 UTM 좌표 구하기
        # transformer = Transformer.from_crs(CRS.from_epsg(4326), CRS.from_epsg(32652), always_xy=True)  # UTM Zone 52
        start_utm_easting, start_utm_northing = self.current_position.x, self.current_position.y
        heading_rad = self.vehicle_yaw

        delta_utm_easting = self.delta_x * math.cos(heading_rad) - self.delta_y * math.sin(heading_rad)
        delta_utm_northing = self.delta_x * math.sin(heading_rad) + self.delta_y * math.cos(heading_rad)
        
        # 시작 위치에 UTM 좌표 변화량을 더하여 최종 UTM 좌표 구하기
        end_utm_easting = start_utm_easting + delta_utm_easting
        end_utm_northing = start_utm_northing + delta_utm_northing

        return end_utm_easting, end_utm_northing

    
    #------------------heading각 계산----------------------------

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

    def calculate_bounding_box_center(self, bev_coords):
        center_x = (bev_coords[0] + bev_coords[2] + bev_coords[4] + bev_coords[6]) / 4
        center_y = (bev_coords[1] + bev_coords[3] + bev_coords[5] + bev_coords[7]) / 4
        return center_x, center_y

    def calculate_bounding_box_dimensions(self, bev_coords):
        width = math.sqrt((bev_coords[2] - bev_coords[0]) ** 2 + (bev_coords[3] - bev_coords[1]) ** 2)
        height = math.sqrt((bev_coords[4] - bev_coords[2]) ** 2 + (bev_coords[5] - bev_coords[3]) ** 2)
        return width, height

    def calculate_angle_with_vehicle(self, center_x, center_y, vehicle_x, vehicle_y):
        angle_rad = math.atan2(center_y - vehicle_y, center_x - vehicle_x)
        angle_deg = math.degrees(angle_rad)
        return angle_deg

    def calculate_distance_to_vehicle(self, center_x, center_y, vehicle_x, vehicle_y):
        distance = math.sqrt((center_x - vehicle_x) ** 2 + (center_y - vehicle_y) ** 2)
        return distance


def run():
    rospy.init_node("gps2utm")
    new_classs= GPS2UTM()
    rospy.spin()
    

if __name__ == '__main__':
    run()