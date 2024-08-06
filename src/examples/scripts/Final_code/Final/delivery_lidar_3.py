#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
# import tf
import os
import rospkg
from std_msgs.msg import Float32MultiArray,Bool,Int32,Int64
from sensor_msgs.msg import Imu
# from morai_msgs.msg import GPSMessage
from geometry_msgs.msg import Point,Twist,TwistStamped
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

class DELIVERY:
    def __init__(self):
        rospy.loginfo("GPS2UTM is Created")
        rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        rospy.Subscriber("odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber("/traffic_sign",Int32,self.vision_callback)
        rospy.Subscriber("/vel",TwistStamped,self.velocity_callback)
        
        self.marker_pub = rospy.Publisher('markers', MarkerArray, queue_size=1)  # Define the marker publisher
        self.marker1_pub = rospy.Publisher('utm_marker', Marker, queue_size=1)  # Define the marker publisher
        self.delivery_utm_pub=rospy.Publisher('utm_sign', Float32MultiArray, queue_size=1)
        self.delivery_stop_pub=rospy.Publisher('/observed_sign', Bool, queue_size=1)
        
        #----------------변수 초기화---------------------
        self.temp1=self.temp2= self.tempa=self.tempb=0
        self.wheelbase=1.44
        self.is_gps=False
        self.current_time=0
        self.UTM= None
        #----------------------------------------------       
        self.vehicle_yaw=0.
        self.vel=float("inf")
        self.is_odom=False
        self.flag=False
        self.current_position=Point()
        self.marker_array = MarkerArray()
        self.Delivery_target_idx = None # A에 따른 목표 B number
        self.Delivery_End=False
        self.utm_end = False
        # self.vision_num=None
        self.rospack = rospkg.RosPack()

    
    def odom_callback(self,msg):    
        self.is_odom=True
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y
        self.vehicle_yaw=msg.pose.pose.position.z
    
    def vision_callback(self,vision_sign):

        #### Delivery: topic from vision ####
        # A1:11, A2:12, A3:13 , idx = B1:0, B2:1, B3:2
        self.vision_num = vision_sign.data

        # self.vision_num =12
        if self.vision_num==11 or self.vision_num==12 or self.vision_num==13 and self.flag==False:
            self.Delivery_target_idx = {11:0, 12:1, 13:2}.get(self.vision_num,None)
            print("=================")
            print("=================")
            print("=================")
            print("=================")
            if self.Delivery_target_idx is not None: 
                self.flag=True
        

    def velocity_callback(self, vel):
        self.is_vel=True
        self.vel=np.sqrt((vel.twist.linear.x)**2+(vel.twist.linear.y)**2)
        if np.isnan(self.vel):
            self.vel=0
        # print("vel")
        # print("self.vel",self.vel)


    def lidar_callback(self, _data):   
        total_obj_cnt = _data.size
        delivery_signs=[]

        # print("=================")
        # print("total_cnt")
        # print(total_obj_cnt)
        # print("=================")
        # print("rate")
        # print(1/(time.time()-self.current_time))

        delivery_sign_cnt=0
        self.current_time=time.time()
        self.marker_array = MarkerArray()
        print("---------------------------------")
        print("Delivery_target_idx",self.Delivery_target_idx)
        
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
         
            x=bbox_center_x 
            y=bbox_center_y

            # 바꾸기 
            if(0<x<15 and -4<y<1.3 and 0.74>bbox_height>0.3 and 0.9>bbox_width>0.2):
            # if(x<14 and -4<y<1.3 and 0.25>bbox_height>0.09 and 0.2>bbox_width>0.09): # 검증용

                print("x,y",x,y,"W",bbox_width,"H",bbox_height)
                print("obs_x: {:.6f}, obs_y: {:.6f}".format(end_utm_easting,end_utm_northing))
                delivery_signs.append((x,y,end_utm_easting,end_utm_northing))
                delivery_sign_cnt+=1
                # self.add_marker(_data.header,"map",end_utm_easting,end_utm_northing,delivery_sign_cnt)
                #####
                
    
        print("**************")
        print("cnt",delivery_sign_cnt)
        print("**************")

        # Delivery_stop_point
        msg=Bool()
        # roi랑 비전 감지로 stop 영역 설정 

        if delivery_sign_cnt>=3 and (self.vision_num==21 or self.vision_num==22 or self.vision_num==23): # 플래너에 정지 명령 
            msg=True
            print("==============")
            print("Delivery start")
            print("Delivery start")
            print("=========")
            delivery_signs.sort(key=lambda x: x[0])  # x
            self.Delivery_End = True

        elif self.Delivery_End==True: # True 한번 잡히면 계속 True만 발행 
            msg=True
        else:
            msg=False
        
        self.delivery_stop_pub.publish(msg)
     
        #####delivery utm coordinate calculate#####
        if self.is_odom and self.Delivery_target_idx is not None and len(delivery_signs)>=3 and self.Delivery_End==True and self.vel==0 and self.utm_end == False:
        # if self.is_odom and self.Delivery_target_idx is not None and len(delivery_signs)>=3 and self.Delivery_End==True and self.utm_end == False:
                
                print("DELIVERY target-idx",self.Delivery_target_idx)

                position_x = self.current_position.x
                position_y = self.current_position.y

                pkg_name = 'examples' 

                # 바꾸기
                path_name = "please_delivery"   # 경로 바꾸기
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
                
                self.delivery_zone_x = delivery_signs[self.Delivery_target_idx][2]
                self.delivery_zone_y = delivery_signs[self.Delivery_target_idx][3]

                target_utm = self.generate_delivery_path(position_x, position_y, delivery_line_x, delivery_line_y, self.delivery_zone_x, self.delivery_zone_y)
                print("target_utm",target_utm)
                print("==================")
                print("1111111111111111111")
                print("==================")

                # 바꾸기 
                if  302602.7477251531 < target_utm[0] <302605.88630547404 and 4124014.3089708383 < target_utm[1] < 4124045.817027482:
                    self.utm_end = True
                    self.UTM =  [target_utm[0],target_utm[1]] 

                    #### Delivery target_utm check marker ####
                    # self.add_marker( _data.header,"map",target_utm[0],target_utm[1],1000) 

                    print("==================")
                    print("2222222222222222")
                    print("==================")
                       
        else:
            print("Not calculate delivery")
        
        print("self.UTM",self.UTM)

        if self.UTM is not None :
            target_utm_msg = Float32MultiArray()
            target_utm_msg.data = self.UTM  # You can put any other 32-bit float numbers here   
            self.delivery_utm_pub.publish(target_utm_msg) 

            marker1 = Marker()
            marker1.header = _data.header
            marker1.header.frame_id = "map"
            marker1.type = Marker.SPHERE  # You can choose the marker type you prefer
            marker1.action = Marker.ADD
            marker1.pose.position.x = self.UTM[0]
            marker1.pose.position.y = self.UTM[1]
            marker1.pose.position.z = 1.0
            marker1.scale.x = 1.0  # Adjust the marker size as needed
            marker1.scale.y = 1.0
            marker1.scale.z = 1.0
            marker1.color.a = 1.0  # Alpha (transparency)
            marker1.color.r = 1.0  # Red color
            marker1.color.g = 0.0  # Green color
            marker1.color.b = 0.0  # Blue color
            marker1.id=100
            self.marker1_pub.publish(marker1)

        # custom_msg/PointArray_msg.msg 메시지 생성
        self.marker_pub.publish(self.marker_array)
        pass


    def generate_delivery_path(self, x, y, delivery_line_x, delivery_line_y, delivery_zone_x, delivery_zone_y):
    #    x 302605.233589
    #    y: 4124033.73486
        distance_list = []

        for i in range(len(delivery_line_x)):
            distance = sqrt(pow(delivery_line_x[i] - delivery_zone_x,2) + pow(delivery_line_y[i] - delivery_zone_y,2))
            distance_list.append(distance)

        minimum_distance = min(distance_list)
        minimum_distance_index = distance_list.index(minimum_distance)

        target_utm=(delivery_line_x[minimum_distance_index],delivery_line_y[minimum_distance_index])

        return target_utm


    def cal_obs_data(self,delta_x,delta_y):
        x=delta_x            
        y=delta_y
        obs_angle = np.rad2deg(math.atan2(y,x))
        obs_dist = np.sqrt(x**2+y**2)

        return obs_angle, obs_dist
        
    def calculate_longitude_latitude(self):
        # 시작 위치의 UTM 좌표 구하기
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
    rospy.init_node("Delivery")
    new_classs= DELIVERY()
    rospy.spin()
    

if __name__ == '__main__':
    run()