#!/usr/bin/env python2
# # -*- coding: utf-8 -*-

import rospy
# import tf
import os
# from std_msgs.msg import Float32MultiArray
# from sensor_msgs.msg import Imu
# from morai_msgs.msg import GPSMessage
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from pyproj import Proj
from tracking_msg.msg import TrackingObjectArray
from visualization_msgs.msg import Marker, MarkerArray
# from scipy.spatial import distance
import pyproj
from math import pi,sqrt
import math
from geometry_msgs.msg import Quaternion
from pyproj import CRS, Transformer
from geometry_msgs.msg import PoseArray, Pose
from custom_msg.msg import PointArray_msg # geometry_msgs/Point[] array -> custom msg 생성 
import numpy as np
import time
from std_msgs.msg import Int32, Bool,String

#############
#### 예선 ####
#############

class GPS2UTM:
    def __init__(self):
        rospy.loginfo("Uturn is Created")

        # ------------------------- Subscriber ----------------------
        rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        # rospy.Subscriber("/State",String,self.state_callback)

        # -------------------------- Marker ----------------------
        self.middle_point_pub = rospy.Publisher("middle_point", Marker, queue_size=10)
        self.lane_point_pub = rospy.Publisher("lane_point", Marker, queue_size=10)
        # ------------------------- Publisher ----------------------------
        self.target_point_publisher = rospy.Publisher("avoid_point", Float32MultiArray, queue_size=10)
        self.obstacle_state_pub = rospy.Publisher('obstacle_state', String, queue_size=5)
        self.bev_pub = rospy.Publisher('bev', PoseArray, queue_size=10)
        # self.uturn_pub = rospy.Publisher('traffic_labacon', Bool, queue_size=10)

        # U-turn 
        self.rubber_cnt=0
        self.ROI_1st_cnt=0
        self.State=None
        # self.stable=False

    # def state_callback(self,state):
    #     self.State=state.data
     
    def lidar_callback(self, _data):
        
        total_obj_cnt = _data.size                          
        self.L_closet_obs1=None
        self.rubber_cnt=0
        # UTM 좌표들을 저장할 빈 리스트 생성
        pointcloud = []
        obstacle_list = []
        
        self.current_time=time.time()
        
        bev_msg = PoseArray()
        bev_msg.header = _data.header
        
        obj = _data.array
        
        uturn_cone=[]
        
        # not in tunnel, return 
        # if not self.current_waypoint in self.tunnel_waypoint:
        #     return
     
        # ------------------- processing roi data --------------------
        for i, obj in enumerate(obj):

            if len(obj.bev.data) < 8:
                rospy.logwarn("Invalid bounding box format: %s", obj.bev.data)
                continue

            bbox_center_x, bbox_center_y = self.calculate_bounding_box_center(obj.bev.data)
            bbox_width, bbox_height = self.calculate_bounding_box_dimensions(obj.bev.data)

            # if(-7<bbox_center_y<0 and 4<bbox_center_x<6 and bbox_width<0.2 and 0.1<bbox_height<0.5): #라바콘 후보군 count
            if(-0.5<bbox_center_y<7 and 3<bbox_center_x<7 and 0.03<bbox_width<0.2 and 0.1<bbox_height<0.5): #라바콘 후보군 count
                self.rubber_cnt+=1
                # print("bbox_width",bbox_width)
            # if(bbox_center_x<2 and 0.05<bbox_width<0.2):
            #     print("bbox_width",bbox_width,bbox_center_x,bbox_center_y)
            # d = distance.euclidean((bbox_center_x,bbox_center_y),(0,0)) # 라이다로부터 물체까지 거리 
            _,d = self.cal_obs_data(bbox_center_x,bbox_center_y)

            ##################### U-turn ##################
            if(self.State=="Rubber_cone_drive"):
                # 초반에는 오른쪽 ROI의 라바콘만 기준으로 잡기 
                # if(0<=self.ROI_1st_cnt<3 and 1<bbox_center_x<6 and 0<bbox_center_y<-3):
                if(0<=self.ROI_1st_cnt<20 and 0.4<bbox_center_x<7 and 0<bbox_center_y<4):
                    uturn_cone.append((d,bbox_center_x,bbox_center_y))
                    print("11111111111111")
                elif(20<self.ROI_1st_cnt and d<5):
                    print("2"*30)
                    uturn_cone.append((d,bbox_center_x,bbox_center_y))
            ###############################################
            
            if self.front_roi[0] < bbox_center_x < self.front_roi[1]:
                if self.side_roi[0] < bbox_center_y <self.side_roi[1]:
                    obstacle_list.append([bbox_center_x, bbox_center_y, d])

        ###################################################################
        # U-turn FLAG
        print("rubber_cnt1",self.rubber_cnt)
        # msg=Bool()
        # if(self.rubber_cnt>=4):
        #     print("U-turn")
        #     print("U-turn")
        #     print("U-turn")
        #     print("U-turn")
        #     msg.data=True
        #     self.uturn_pub.publish(msg)
        # else: 
        #     msg.data=False
        #     # print(msg.data)
        #     self.uturn_pub.publish(msg)

        uturn_cone.sort(key=lambda x: x[0])

        if(self.State=="Rubber_cone_drive"):
            self.ROI_1st_cnt+=1
            print("ROI_1st_cnt",self.ROI_1st_cnt)
            if(0<self.ROI_1st_cnt<20):
                lane_point=(uturn_cone[0][1],uturn_cone[0][2])
                mid_point=(lane_point[0],lane_point[1]+2.5)
            else:
                lane_point=(uturn_cone[0][1],uturn_cone[0][2])
                mid_point=(lane_point[0],lane_point[1]+2)

            print("===================")
            print("mid_point",mid_point)

            target_point=Float32MultiArray()
            target_point.data.append(mid_point[0])
            target_point.data.append(mid_point[1])
            self.target_point_publisher.publish(target_point)

            self.publish_obstacles(mid_point, self.middle_point_pub, color=(0.0, 1.0, 0.0))  # 초록색으로 시각화
            self.publish_obstacles(lane_point, self.lane_point_pub, color=(1.0, 0.0, 0.0))  # 초록색으로 시각화
        ###############################################################################


    def publish_obstacles_to_array(self,point, marker_array, color=(0.0, 0.0, 1.0), marker_id=0):
        marker = Marker()
        marker.header.frame_id = "velodyne"  # 프레임 ID 설정
        marker.type = marker.SPHERE  # 마커 형태
        marker.action = marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 0
        marker.scale.x = 0.8
        marker.scale.y = 0.8
        marker.scale.z = 0.8
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.id = marker_id  # 마커 ID 설정
        
        marker_array.markers.append(marker)  # MarkerArray에 마커 추가
    
    def cal_obs_data(self,delta_x,delta_y):
        x=delta_x
        y=delta_y

        obs_angle = np.rad2deg(math.atan2(y,x))
        obs_dist = np.sqrt(x**2+y**2)
        
        return obs_angle, obs_dist
    
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
            marker.scale.x = 0.6  # 포인트 크기
            marker.scale.y = 0.6
            marker.scale.z = 0.6
            marker.color.a = 1.0
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]

            publisher.publish(marker)



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
    rospy.init_node("uturn")
    new_classs= GPS2UTM()
    rospy.spin()
    

if __name__ == '__main__':
    run()
