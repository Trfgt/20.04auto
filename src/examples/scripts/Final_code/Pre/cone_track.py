#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import rospkg
from std_msgs.msg import Float32MultiArray,Bool,Int32,Int64
from sensor_msgs.msg import Imu
# from morai_msgs.msg import GPSMessage
from geometry_msgs.msg import Point,Twist
from tracking_msg.msg import TrackingObjectArray
from math import pi,sqrt
import math
from visualization_msgs.msg import Marker, MarkerArray
from custom_msg.msg import PointArray_msg # geometry_msgs/Point[] array -> custom msg 생성 
import numpy as np
import time
from nav_msgs.msg import Odometry
from vehicle_msgs.msg import Track, TrackCone


####################
#### Cone Track ####
####################

class Conetrack:
    def __init__(self):
        rospy.loginfo("Conetrack is Created") 
        rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        self.cone_pub = rospy.Publisher('/track',Track, queue_size=1)

        #----------------변수 초기화---------------------
        self.temp1=self.temp2= self.tempa=self.tempb=0
        self.wheelbase=1.44
        self.is_gps=False
        self.current_time=0
        #----------------------------------------------       
        self.vehicle_yaw=0.
        self.is_odom=False
        self.flag=False
        self.current_position=Point()
        self.marker_array = MarkerArray()

        rate = rospy.Rate(10) # 20hz
        self.rospack = rospkg.RosPack()

    
    # def odom_callback(self,msg):    
    #     self.is_odom=True
    #     self.current_position.x=msg.pose.pose.position.x
    #     self.current_position.y=msg.pose.pose.position.y
    #     self.vehicle_yaw=msg.pose.pose.position.z
    
   
    def velocity_callback(self, vel):
        self.vel=vel.twist.linear.x
        if np.isnan(self.vel):
            self.vel=0


    def lidar_callback(self, _data):   
        total_obj_cnt = _data.size

        # print("=================")
        # print("total_cnt")
        # print(total_obj_cnt)
        # print("=================")
        # print("rate")
        # print(1/(time.time()-self.current_time))

        self.current_time=time.time()
        self.marker_array = MarkerArray()
        self.cones=Track()
        cnt = 0 
        for obj in _data.array:

            if len(obj.bev.data) < 8:
                rospy.logwarn("Invalid bounding box format: %s", obj.bev.data)
                continue

            bbox_center_x, bbox_center_y = self.calculate_bounding_box_center(obj.bev.data)
            bbox_width, bbox_height = self.calculate_bounding_box_dimensions(obj.bev.data)

            x=bbox_center_x 
            y=bbox_center_y

            if(10>x>0 and 0.09<bbox_height<0.35 and 0.05<bbox_width<0.25):
                cone = TrackCone()

                # print("x,y",x,y,"W",bbox_width,"H",bbox_height)
                cone.x=x
                cone.y=y
                cnt+=1
                
                self.cones.cones.append(cone)

        print("cnt",cnt)

        self.cone_pub.publish(self.cones)
        
       
    def cal_obs_data(self,delta_x,delta_y):
        x=delta_x            
        y=delta_y
        obs_angle = np.rad2deg(math.atan2(y,x))
        obs_dist = np.sqrt(x**2+y**2)

        return obs_angle, obs_dist
    
        
    def calculate_bounding_box_dimensions(self, bev_coords):
        width = math.sqrt((bev_coords[2] - bev_coords[0]) ** 2 + (bev_coords[3] - bev_coords[1]) ** 2)
        height = math.sqrt((bev_coords[4] - bev_coords[2]) ** 2 + (bev_coords[5] - bev_coords[3]) ** 2)
        return width, height


    def calculate_bounding_box_center(self, bev_coords):
        center_x = (bev_coords[0] + bev_coords[2] + bev_coords[4] + bev_coords[6]) / 4
        center_y = (bev_coords[1] + bev_coords[3] + bev_coords[5] + bev_coords[7]) / 4
        return center_x, center_y


def run():
    rospy.init_node("Conetrack")
    new_classs= Conetrack()
    rospy.spin()
    

if __name__ == '__main__':
    run()