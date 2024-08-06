#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
# import tf
import os
# from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from pyproj import Proj
from tracking_msg.msg import TrackingObjectArray
from visualization_msgs.msg import Marker, MarkerArray
import pyproj
from math import pi,sqrt
import math
from geometry_msgs.msg import Quaternion
from pyproj import CRS, Transformer
from geometry_msgs.msg import PoseArray, Pose
from custom_msg.msg import PointArray_msg # geometry_msgs/Point[] array -> custom msg 생성 
import numpy as np
import time


class GPS2UTM:
    def __init__(self):
        rospy.loginfo("GPS2UTM is Created")
        rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        self.middle_point_pub = rospy.Publisher("middle_point", Marker, queue_size=1)
        self.target_point_publisher = rospy.Publisher("target_point", Float32MultiArray, queue_size=10)
        self.bev_pub = rospy.Publisher('bev', PoseArray, queue_size=10)
        #----------------변수 초기화---------------------
        self.temp1=self.temp2= self.tempa=self.tempb=0
        self.wheelbase=1.1
        self.is_gps=False
        self.current_time=0
        #----------------------------------------------
        self.closet_obs1=None
        self.closet_obs2=None
        
        #-----충돌 가능성 판단----------------------------
        self.warning_x=5
        self.warning_y=5
     
    def gps_callback(self,_data):
        self.start_longitude = _data.longitude
        self.start_latitude = _data.latitude
        self.is_gps=True
        pass
    
    def imu_callback(self, _data):
        self.Orientation = _data.orientation
        # rospy.loginfo(self.Orientation)
        # rospy.loginfo("1, %.2f",self.y_vel)
        # rospy.loginfo(self.y_vel
        self.heading = self.calculate_heading_from_imu()
        # print(f"Heading: {self.heading}")
        pass
    
    
    def lidar_callback(self, _data):
        
        total_obj_cnt = _data.size
        self.closet_obs1=None
        self.closet_obs2=None

        # UTM 좌표들을 저장할 빈 리스트 생성
        pointcloud = []
        east_offset = 302459.942
        north_offset = 4122635.537
        # print(total_obj_cnt)

        # print(1/(time.time()-self.current_time))
        
        self.current_time=time.time()

        bev_msg = PoseArray()
        bev_msg.header = _data.header
        min_distance_1 = float('inf')
        min_distance_2 = float('inf')
        
        for i, obj in enumerate(_data.array):
            if len(obj.bev.data) < 8:
                rospy.logwarn("Invalid bounding box format: %s", obj.bev.data)
                continue
            # exception
            bbox_center_x, bbox_center_y = self.calculate_bounding_box_center(obj.bev.data)
            bbox_width, bbox_height = self.calculate_bounding_box_dimensions(obj.bev.data)
            # ㅡㅡㅡㅡㅡㅡㅡ
            # |         | ^
            # | warning | |
            # |         | 2m
            # |<--2m--->| v
            # ㅡㅡlidarㅡㅡ
            print(bbox_center_x,bbox_center_y)
            if(-self.warning_x<bbox_center_x<self.warning_x and -self.warning_y<bbox_center_y<self.warning_y):
                # self.crash=True
                print("------------------------------------------")
                print("cone VS drum bbox width",bbox_width)
                print("------------------------------------------")
    
            self.delta_x = bbox_center_x + self.wheelbase
            self.delta_y = bbox_center_y
            # self.delta는 라이다와 gps 위치 차이 고려
            # wheelbase는 약 1.1로 설정. 부착위치에 따라 변경 요구

            x=bbox_center_x 
            y=bbox_center_y
            # 라이다 고유 데이터
            obs_angle,obs_dist=self.cal_obs_data(x,y)
            print(obs_dist)

            # 1st closest
            if(obs_dist<min_distance_1 and (0<x<8 and -3<y<0.5)):
                
                # before update, compare to 2nd data
                if(self.closet_obs1 is not None):
                    
                    _,last_short_distance1 = self.cal_obs_data(self.closet_obs1[0],self.closet_obs1[1])
                    if(last_short_distance1<min_distance_2):
                        self.closet_obs2=self.closet_obs1
                        min_distance_2=min_distance_1
                        
                self.closet_obs1=(x,y)
                min_distance_1=obs_dist
                

            # 2nd closest
            if(min_distance_1<obs_dist<=min_distance_2 and(0<x<8 and -3<y<0.5)):
                self.closet_obs2=(x,y)
                min_distance_2=obs_dist
            
            
        print("=======================================")
        print("closet_1",self.closet_obs1,"dist_1",min_distance_1)
        print("closet_2",self.closet_obs2,"dist_2",min_distance_2)
        print("=======================================")
        
        if(self.closet_obs2 is not None):
            # gradient=(self.closet_obs1[0]-self.closet_obs2[0])/(self.closet_obs1[1]-self.closet_obs2[1])
            # print("gradient",gradient)
            
            #-----------1. Get target point from 2nd closest point-------------
            mid_point=(self.closet_obs2[0]/2,self.closet_obs2[1]/2)
        
            #-----------2. Difference of closest two points--------------------
            # mid_point=(self.closet_obs2[0]-self.closet_obs1[0],self.closet_obs2[1]-self.closet_obs1[1])
            print("mid_x,y",mid_point)
            
            self.publish_obstacles(mid_point, self.middle_point_pub, color=(0.0, 1.0, 0.0))  # 초록색으로 시각화
            # print()
            target_point=Float32MultiArray()
            target_point.data.append(mid_point[0])
            target_point.data.append(mid_point[1])
            self.target_point_publisher.publish(target_point)
        
            # angle이랑 dist 계산
            # angle은 현재 degree 상태. -> cal_obs_data에 수정 가능.

            # end_utm_easting, end_utm_northing = self.calculate_longitude_latitude()
            # print("obs_x: {:.6f}, obs_y: {:.6f}".format(end_utm_easting - east_offset,end_utm_northing - north_offset))
             # UTM 좌표를 Point 메시지로 변환하여 리스트에 추가
            
            # if obs_dist <= 5:
            #     print("============obs_data==================")
            #     print("obs_x: {:.6f}, obs_y: {:.6f}".format(end_utm_easting - east_offset,end_utm_northing - north_offset))
            #     print("obs_angle: {:.3f}, obs_dist: {:.3f}".format(obs_angle,obs_dist))
            #     print("obs_width: {:.3f}, obs_height: {:.3f}".format(bbox_width,bbox_height))

            #     bev_pose = Pose()
            #     bev_pose.position.x = bbox_center_x
            #     bev_pose.position.y = bbox_center_y
            #     bev_pose.position.z = 0.0

            #     bev_pose.orientation.x = bbox_width
            #     bev_pose.orientation.y = bbox_height
            #     bev_pose.orientation.z = obs_dist
            #     bev_pose.orientation.w = obs_angle

            #     bev_msg.poses.append(bev_pose)
            # dist <= 5이하일 때만 장애물로 추가.
            

        # custom_msg/PointArray_msg.msg 메시지 생성
        self.bev_pub.publish(bev_msg)

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
    
    # def publish_obstacles(self, x1,y1, publisher, color):
    #     if x1 is not None:
    #         x, y = x1,y1
    #         # Marker 메시지를 생성하여 장애물들을 크고 입체적으로 시각화
    #         marker = Marker()
    #         marker.header.frame_id = "velodyne"  # 필요에 따라 적절한 프레임으로 변경
    #         marker.header.stamp = rospy.Time.now()
    #         marker.ns = "obstacles"
    #         marker.id = 0
    #         marker.type = Marker.SPHERE
    #         marker.action = Marker.ADD
    #         marker.pose.position.x = x
    #         marker.pose.position.y = y
    #         marker.pose.position.z = 0.0  # 입체적으로 보이기 위해 z 좌표를 0 이상으로 설정
    #         marker.scale.x = 1.0  # 포인트 크기
    #         marker.scale.y = 1.0
    #         marker.scale.z = 1.0
    #         marker.color.a = 1.0
    #         marker.color.r = color[0]
    #         marker.color.g = color[1]
    #         marker.color.b = color[2]

    #         publisher.publish(marker)
    
    def publish_obstacles(self, obs, publisher, color):
        if obs is not None:
            x, y = obs[0],obs[1]
            # Marker 메시지를 생성하여 장애물들을 크고 입체적으로 시각화
            marker = Marker()
            marker.header.frame_id = "velodyne"  # 필요에 따라 적절한 프레임으로 변경
            marker.header.stamp = rospy.Time.now()
            marker.ns = "obstacles"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0  # 입체적으로 보이기 위해 z 좌표를 0 이상으로 설정
            marker.scale.x = 1.0  # 포인트 크기
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.a = 1.0
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]

            publisher.publish(marker)
        
    def calculate_longitude_latitude(self):
        # 시작 위치의 UTM 좌표 구하기
        transformer = Transformer.from_crs(CRS.from_epsg(4326), CRS.from_epsg(32652), always_xy=True)  # UTM Zone 52
        start_utm_easting, start_utm_northing = transformer.transform(self.start_longitude, self.start_latitude)
        
        # 차량 UTM 좌표
        # print("Start UTM Easting:",start_utm_easting, "Start UTM Northing:", start_utm_northing)

        # 프레임별 차량 UTM좌표 차이 
        # print("----start", start_utm_easting-self.tempa, start_utm_northing-self.tempb)
        
        # 헤딩값을 라디안 단위로 변환
        heading_rad = math.radians(self.heading)
        # print("heading_rad", heading_rad*180/pi)

        self.tempa= start_utm_easting
        self.tempb= start_utm_northing
        

        # 상대적인 x와 y 위치를 UTM 좌표계로 변환
        delta_utm_easting = self.delta_x * math.cos(heading_rad) - self.delta_y * math.sin(heading_rad)
        delta_utm_northing = self.delta_x * math.sin(heading_rad) + self.delta_y * math.cos(heading_rad)
        
        # print(heading_rad)
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
        # print("============")
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
