#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point,TwistStamped
from tracking_msg.msg import TrackingObjectArray
# from gps import GPS2UTM
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import numpy as np
from math import atan,cos,sin,tan
import time
from std_msgs.msg import Int32

# 주차 표지판 받았을 때 실행 

class LidarReceiver():
    def __init__(self):
        # super().__init__()
        rospy.loginfo("LiDAR Receiver Object is Created")
        rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        rospy.Subscriber("/odom_gps", Odometry, self.odom_callback)
        rospy.Subscriber("/current_waypoint", Int32, self.index_callback)
        rospy.Subscriber("/vel",TwistStamped,self.velocity_callback)

        self.marker_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        self.parking_lot_pub = rospy.Publisher('parking_plot', Int32, queue_size = 1)
        self.array=[]

        self.vel=float("inf")
        self.is_odom=False
        self.is_index = False
        self.current_position = Point()

        self.current_time = time.time()
        self.parking_plan_index = np.arange(1765, 1780) # 본선
        # self.parking_plan_index = np.arange(1183,1215)
        self.accuracy_check=0
        self.before_num = float("inf")

        self.LW=1.44
        self.calculate_idx = 0
        self.parking_num = -1
    
    def velocity_callback(self, vel):
        self.is_vel=True
        self.vel=np.sqrt((vel.twist.linear.x)**2+(vel.twist.linear.y)**2)
        if np.isnan(self.vel):
            self.vel=0

    def index_callback(self, msg):
        self.is_index = True
        self.index=msg.data
    
    def odom_callback(self,msg):
        self.is_odom=True
        self.current_position.x=msg.pose.pose.position.x
        self.current_position.y=msg.pose.pose.position.y
        self.vehicle_yaw = msg.pose.pose.position.z
        
    
    def lidar_callback(self, msg):

        if self.is_odom==False or self.is_index==False and self.vel!=0 : #본선 
        # if self.is_odom==False or self.is_index==False :
            pass
        else:
            if not self.index in self.parking_plan_index:
                print("NOT IN PARKING_TRIGGER!!")
                return
            
            if self.calculate_idx > 10:
                print("COMPLETE CALCULATE PARKING_LOT_NUMBER")
                self.parking_lot_pub.publish(self.parking_num)
                return
            
            self.calculate_idx += 1
            
            rate = 1/(time.time()-self.current_time)
            # print(rate)
            self.current_time = time.time()
            self.marker_array = MarkerArray()
            parking_lots = [
                # ### 바꾸기 (본선)
                [(302478.09375,4123758.25), (302480.5625,4123763.0), (12.6, 1082.65), (7.96, 1080.9)],  # 구역 1  
                [(302475.1875,4123753.75), (302478.09375,4123758.25), (14.54, 1086.03), (9.81, 1084.35)], # 구역 2
                [(302473.0,4123749.5), (302475.1875,4123753.75), (16.31, 1089.32), (11.65, 1087.61)] # 구역 3 
                
                # 302480.5625,4123763.0 # 1
                # 302478.09375,4123758.25 # 2
                # 302475.1875,4123753.75 #3
                # 302473.0,4123749.5 #4

                ### 바꾸기 (snu)
                # [(298514.22298,4137846.45302), (298511.324785,4137842.70743), (12.6, 1082.65), (7.96, 1080.9)],  # 구역 1  
                # [(298517.37703,4137850.53126), (298514.22298,4137846.45302), (14.54, 1086.03), (9.81, 1084.35)], # 구역 2
                # [(298520.183656,4137854.10567), (298517.37703,4137850.53126), (16.31, 1089.32), (11.65, 1087.61)] # 구역 3 

                # 298511.324785, 4137842.70743 # 1
                # 298514.22298, 4137846.45302 # 2
                # 298517.37703, 4137850.53126 # 3 
                # 298520.183656, 4137854.10567 #4 
            
            ]
             
            obstacle_points = []
            
            for obj in msg.array:
                if len(obj.bev.data) < 8:
                    rospy.logwarn("Invalid bounding box format: %s", obj.bev.data)
                    continue

                bbox_center_x, bbox_center_y = self.calculate_bounding_box_center(obj.bev.data)
                # -> ENU 
                self.delta_x = bbox_center_x
                self.delta_y = bbox_center_y
                end_utm_easting, end_utm_northing = self.calculate_longitude_latitude(self.delta_x,self.delta_y)

                # print(end_utm_easting,end_utm_northing)

                if abs(bbox_center_y)<5. and bbox_center_x < 15.:
                    obstacle_points.append((end_utm_easting, end_utm_northing))
                
                print("Marker the parking lot") # 좌표 바꾸기 
                self.add_marker(msg.header,"map",302480.5625,4123763.0,111111) 
                self.add_marker(msg.header,"map",302478.09375,4123758.25,11121)   
                self.add_marker(msg.header,"map",302475.1875,4123753.75,111113)
                self.add_marker(msg.header,"map",302473.0,4123749.5,12111)
                
                # 302480.5625,4123763.0 # 1
                # 302478.09375,4123758.25 # 2
                # 302475.1875,4123753.75 #3
                # 302473.0,4123749.5 #4 
            self.marker_pub.publish(self.marker_array)
            # print(len(obstacle_points),len(parking_lots))
            
            min_distance = np.inf
            nearest_empty_parking_lot = -1

            print("INint!!!!!!!")
            
            for i, parking_lot in enumerate(parking_lots):
                marker = self.create_marker(parking_lot)
                marker.id = i
                is_empty = True

                for obstacle_point in obstacle_points:   # obstacle points -> 장애물

                    is_inside_obstacle = self.is_point_inside_polygon(parking_lot, obstacle_point)

                    print("is_inside:",is_inside_obstacle)

                    if is_inside_obstacle :#and i not in self.array:
                        marker.color.r = 1.0
                        marker.color.g = 0.0
                        is_empty = False
                        self.array.append(i)
                        # print(obstacle_point, 'red', i)
                        # break
                    
                    # elif i in self.array:
                    #     marker.color.r = 1.0
                    #     marker.color.g = 0.0
                    #     is_empty = False
                        
                    # else:
                    #     marker.color.r = 0.0
                    #     marker.color.g = 1.0
                        # print(obstacle_point, 'green', i)

                # marker_array.markers.append(marker)

                print(is_empty)

                if is_empty:
                    print("near_idx: ", i+1)
                else:
                    print("NOT Parkable: ",i+1)
                
                if is_empty:
                    distance = math.sqrt((self.current_position.x - parking_lot[i][0]) ** 2 + (self.current_position.y - parking_lot[i][1]) ** 2)
                    if distance < min_distance:
                        min_distance = distance
                        nearest_empty_parking_lot = i + 1

                    # print(nearest_empty_parking_lot)

            print("Nearset parking_lot number :", nearest_empty_parking_lot)
            self.parking_num = nearest_empty_parking_lot
            
            # # 장애물 마커 추가 (빨간색으로 표시)
            # for j, obstacle_point in enumerate(obstacle_points):
            #     # obstacle_marker = self.create_obstacle_marker(obstacle_point)
            #     obstacle_marker.id = j + len(parking_lots)  # 주차장 marker 개수만큼 ID를 추가
            #     marker_array.markers.append(obstacle_marker)
            # print("배열",self.array)

            # if(self.accuracy_check>=0):
            #     if nearest_empty_parking_lot == self.before_num : self.accuracy_check+=1
            #     else: self.accuracy_check=0
            
            # if(self.accuracy_check>5):
            #     print("FINAL RESULT NUM")
            #     print(self.before_num)  
            #     self.marker_pub.publish(marker_array)
            #     self.parking_lot_pub.publish(self.before_num)
            #     self.accuracy_check = -1 # 확신 flag

            # elif self.accuracy_check < 0 : # 확실할 때부터 고정값만 발행
            # self.marker_pub.publish(marker_array)
            #     print("FINAL RESULT NUM_CONTINUE")
            #     print(self.before_num)
            #     self.parking_lot_pub.publish(self.before_num)

            # else:
            #     print("READY")
            #     self.before_num = nearest_empty_parking_lot


      ############################################################################################

    def calculate_bounding_box_center(self, bev_coords):
        # bev_coords: [x1, y1, x2, y2, x3, y3, x4, y4]
        center_x = (bev_coords[0] + bev_coords[2] + bev_coords[4] + bev_coords[6]) / 4
        center_y = (bev_coords[1] + bev_coords[3] + bev_coords[5] + bev_coords[7]) / 4
        return center_x, center_y

    def add_marker(self, header, frame_id, x, y, id):
        marker1 = Marker()
        marker1.header = header
        marker1.header.frame_id = frame_id
        marker1.type = Marker.SPHERE  # You can choose the marker type you prefer
        marker1.action = Marker.ADD
        marker1.pose.position.x = x
        marker1.pose.position.y = y
        marker1.pose.position.z = 1.0
        marker1.scale.x = 1.0  # Adjust the marker size as needed
        marker1.scale.y = 1.0
        marker1.scale.z = 1.0
        marker1.color.a = 1.0  # Alpha (transparency)
        marker1.color.r = 0.0  # Red color
        marker1.color.g = 0.0  # Green color
        marker1.color.b = 1.0  # Blue color
        marker1.id=id
        self.marker_array.markers.append(marker1)

    def create_marker(self,points):
        marker_msg = Marker()
        marker_msg.header.frame_id = "map"
        marker_msg.type = Marker.LINE_STRIP
        marker_msg.action = Marker.ADD
        marker_msg.scale.x = 0.05
        marker_msg.color.a = 1.0
        marker_msg.color.r = 1.0
        marker_msg.color.g = 0.0
        marker_msg.color.b = 0.0
        marker_msg.id = 0
        
        for i in range(4):
            p = Point()
            p.x = points[i][0] 
            p.y = points[i][1] 
            p.z = 0.0
            marker_msg.points.append(p)

        p = Point()
        p.x = points[0][0]
        p.y = points[0][1] 
        p.z = 0.0
        marker_msg.points.append(p)

        marker_msg.lifetime = rospy.Duration(0)
        marker_msg.action = Marker.ADD
        marker_msg.frame_locked = True
        marker_msg.ns = "parking_lots"
        marker_msg.pose.orientation.w = 1.0
        marker_msg.pose.orientation.x = 0.0
        marker_msg.pose.orientation.y = 0.0
        marker_msg.pose.orientation.z = 0.0

        return marker_msg
    
    # def create_obstacle_marker(self,point):
    #     marker_msg = Marker()
    #     marker_msg.header.frame_id = "map"
    #     marker_msg.type = Marker.SPHERE
    #     marker_msg.action = Marker.ADD
    #     marker_msg.scale.x = 0.5
    #     marker_msg.scale.y = 0.5
    #     marker_msg.scale.z = 0.5
    #     marker_msg.color.a = 1.0
    #     marker_msg.color.r = 1.0  # 장애물 색상을 빨간색으로 설정
    #     marker_msg.color.g = 0.0
    #     marker_msg.color.b = 0.0
    #     marker_msg.pose.orientation.w = 1.0
    #     marker_msg.pose.position.x = point[0]  # offset_x 값 적용
    #     marker_msg.pose.position.y = point[1]  # offset_y 값 적용

    #     return marker_msg


    def is_point_inside_polygon(self,rectangle_points, point):

        inside = False
        p1x, p1y = rectangle_points[0]
        p2x, p2y = rectangle_points[1]

        center_x=(p1x+p2x)/2
        center_y=(p1y+p2y)/2

        p1x-=center_x; p1y-=center_y
        p2x-=center_x; p2y-=center_y

        grad_, y_intercept=np.polyfit([p1x,p2x],[p1y,p2y],1)

        theta=atan(grad_)

        x=point[0]-center_x; y=point[1]-center_y


        x_2=x*cos(theta)+y*sin(theta)
        y_2=-x*sin(theta)+y*cos(theta)


        x_rotated=p1x*cos(theta)+p1y*sin(theta)
        y_rotated=-p1x*sin(theta)+p1y*cos(theta)

        # print(x,y)

        # print(x_rotated,0)
        # # print(theta)
        # print(x_2,y_2)

        if abs(x_2)<abs(x_rotated):
            # print("===============================")
            # # print(theta,self.vehicle_yaw)
            # print("x_dist: {:.2f}".format(x_rotated*0.8))
            # print("origin_x: {:.2f}, origin_y: {:.2f}".format(x,y))
            # print("x: {:.2f}, y: {:.2f}".format(x_2,y_2))
            # print("===============================")
            pass

        if abs(y_2)<1. and abs(x_2)<abs(x_rotated)*0.8:
            # print("===============================")
            # print("x_dist: {:.2f}".format(x_rotated*0.8))
            # print("x: {:.2f}, y: {:.2f}".format(x_2,y_2))
            # print("===============================")
            inside=True   

        return inside

            

    def calculate_longitude_latitude(self,delta_x,delta_y):
        
        heading_rad = self.vehicle_yaw
        # print("heading_rad", heading_rad)

        delta_x+=self.LW

        delta_utm_easting = delta_x * math.cos(heading_rad) - delta_y * math.sin(heading_rad)
        delta_utm_northing = delta_x * math.sin(heading_rad) + delta_y * math.cos(heading_rad)

        end_utm_easting = self.current_position.x + delta_utm_easting
        end_utm_northing = self.current_position.y + delta_utm_northing

        return end_utm_easting, end_utm_northing
    
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


if __name__ == '__main__':
    try:
        rospy.init_node('parking_plot_recognition', anonymous=True)
        rospy.loginfo("Node started")
        receiver = LidarReceiver()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass