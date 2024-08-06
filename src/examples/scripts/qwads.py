#!/usr/bin/env python3
# # -*- coding: utf-8 -*-

import rospy
# import tf
import os
# from std_msgs.msg import Float32MultiArray
# from sensor_msgs.msg import Imu
# from morai_msgs.msg import GPSMessage
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
from tracking_msg.msg import TrackingObjectArray
from visualization_msgs.msg import Marker, MarkerArray
# from scipy.spatial import distance

from math import pi,sqrt
import math
from geometry_msgs.msg import Quaternion
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
        rospy.Subscriber("/State",String,self.state_callback)

        # -------------------------- Marker ----------------------
        #self.middle_point_pub = rospy.Publisher("middle_point", Marker, queue_size=10)
        self.middle_point_pub = rospy.Publisher('middle_point', Marker, queue_size=10)

        self.lane_point_pub = rospy.Publisher("lane_point", Marker, queue_size=10)
        # ------------------------- Publisher ----------------------------
        self.target_point_publisher = rospy.Publisher("uturn_point", Float32MultiArray, queue_size=10)
        self.obstacle_state_pub = rospy.Publisher('obstacle_state', String, queue_size=5)
        self.bev_pub = rospy.Publisher('bev', PoseArray, queue_size=10)
        # self.uturn_pub = rospy.Publisher('traffic_labacon', Bool, queue_size=10)

        # U-turn 
        self.State="Rubber_cone_drive"
        self.lfirst=False
        self.rfirst=False
        self.left_cones = []
        self.right_cones = []
        self.min_distance_threshold = 3
        self.ltime = time.time()
        self.rtime = time.time()
        # self.stable=False
        self.line_space = 50

    def state_callback(self,state):
        self.State=state.data



    def bezier_curve(self, points, num_point):
        points = np.array(points)
        n = len(points) - 1
        t = np.linspace(0, 1, num_point)
        polynomial_array = np.array([self.bernstein_poly(i, n, t) for i in range(n+1)])
        curve = np.dot(points.T, polynomial_array).T
        return curve
    def comb_math(self, n, k):
        return math.factorial(n) // (math.factorial(k) * math.factorial(n - k))
    def bernstein_poly(self, i, n, t):
        return self.comb_math(n, i) * (t**(n-i)) * (1 - t)**i

    def euclidean_distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    # 새로운 콘을 리스트에 추가할지 여부를 결정하는 함수
    def should_add_cone(self, cones_list, new_cone, min_distance):
        if not cones_list:
            return True
        last_cone = cones_list[-1]
        return self.euclidean_distance(last_cone, new_cone) > min_distance
    def lidar_callback(self, _data):
        
        total_obj_cnt = _data.size    
        self.L_closet_obs1=None
        self.current_time=time.time()
        pointcloud = []
        
        bev_msg = PoseArray()
        bev_msg.header = _data.header

        obj = _data.array
        print("!!!!!!!!!!!!!!!!!!!!!!!")
        print (obj)
        print("!!!!!!!!!!!!!!!!!!!!!!!")
        

        
        self.left_cones= []
        self.right_cones= []

        obj_collector = []
        for i, obj in enumerate(obj):

            if len(obj.bev.data) < 8:
                rospy.logwarn("Invalid bounding box format: %s", obj.bev.data)
                continue
            bbox_center_x, bbox_center_y = self.calculate_bounding_box_center(obj.bev.data)
            bbox_width, bbox_height = self.calculate_bounding_box_dimensions(obj.bev.data)
            if 0.05<bbox_width<0.8 and 0.15<bbox_height<0.8 and bbox_center_x<4:
                obj_collector.append([bbox_center_x, bbox_center_y,bbox_width, bbox_height])
        print(len(obj_collector))
        
        obj_collector.sort(key=lambda x: x[0])
        for bbox_center_x, bbox_center_y,bbox_width, bbox_height in obj_collector:
            if(self.State=="Rubber_cone_drive"):
                new_cone = (bbox_center_x, bbox_center_y)
                #print(new_cone)
                if len(self.left_cones)>10 or len(self.right_cones)>10: 
                    break
                if self.lfirst == False and 0 < bbox_center_x < 7 and -1 < bbox_center_y < 2:
                    self.lfirst == True
                    self.left_cones.append(new_cone)
                elif self.lfirst == True:
                    if self.should_add_cone(self.left_cones, new_cone, self.min_distance_threshold ):
                        self.left_cones.append(new_cone)
                            
                if self.rfirst == False and 0 < bbox_center_x < 7 and -2 < bbox_center_y < 1:
                    self.rfirst == True
                    self.right_cones.append(new_cone)
                elif self.rfirst == True:
                    if self.should_add_cone(self.right_cones, new_cone, self.min_distance_threshold ):
                        self.right_cones.append(new_cone)

        if len(self.left_cones) ==0:
            self.left_cones= [(0,1.1)]
        elif len(self.right_cones) ==0:
            self.right_cones= [(0,-1.1)]
        print(len(self.left_cones),len(self.right_cones))



        if len(self.left_cones) >= 2 and len(self.right_cones) < 2:
            
            left_curve = self.bezier_curve(self.left_cones,self.line_space)
            right_curve = [(row[0], row[1] - 4) for row in left_curve]
        elif len(self.left_cones) < 2 and len(self.right_cones) >= 2:
            right_curve = self.bezier_curve(self.right_cones,self.line_space)
            left_curve = [(row[0], row[1] + 4) for row in right_curve]
        elif len(self.left_cones) < 2 and len(self.right_cones) < 2:
            print("++++++++++++++++++++++++++++++++++++")
            #self.publish_obstacles([1,0],self.middle_point_pub, color=(0.0, 1.0, 0.0)) 

            return

        else:
        
            left_curve = self.bezier_curve(self.left_cones,self.line_space)
            right_curve = self.bezier_curve(self.right_cones,self.line_space)


        uturn_cone=[]
        # 좌우 곡선의 같은 인덱스 점들의 중간점을 계산
        for left, right in zip(left_curve, right_curve):
            if (left[0] + right[0])>0 :
                
                uturn_cone.append([(left[0] + right[0]) / 2, (left[1] + right[1]) / 2])
        
        uturn_cone.sort(key=lambda x: x[0])



        #print(uturn_cone != [])

        if(uturn_cone != [] ):
            #print(np.shape(uturn_cone))

            mid_point = [uturn_cone[int(len(uturn_cone)/5)][0],uturn_cone[int(len(uturn_cone)/5)][1]]
            #print(mid_point)
            target_point=Float32MultiArray()
            target_point.data.append(mid_point[0])
            target_point.data.append(mid_point[1])
            self.target_point_publisher.publish(target_point)
            self.publish_obstacles(mid_point, self.middle_point_pub, color=(0.0, 1.0, 0.0))  

    def cal_obs_data(self,delta_x,delta_y):
        x=delta_x
        y=delta_y

        obs_angle = np.rad2deg(math.atan2(y,x))
        obs_dist = np.sqrt(x**2+y**2)
        
        return obs_angle, obs_dist
    
    def publish_obstacles(self, obs, publisher, color):
        if obs is not None:
            #print(obs)
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
