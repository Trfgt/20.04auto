#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import math
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray
from tracking_msg.msg import TrackingObjectArray
# from gps import GPS2UTM
import numpy as np

class LidarObstacleVisualizer():
    def __init__(self):
        # super().__init__()
        rospy.loginfo("LiDAR Receiver Object is Created")
        self.obstacles_sub = rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.obstacle_callback)
        self.left_point_pub = rospy.Publisher("left_point", Marker, queue_size=1)
        self.text_marker_pub = rospy.Publisher("obstacle_text_markers", MarkerArray, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        self.left_obstacle=None
        self.min_range = 1.5
        self.max_range = 4
     

    def timer_callback(self):
        self.obstacle_callback()

    def obstacle_callback(self, obstacles_msg):
        self.total_obj_cnt = obstacles_msg.size
        for i, obj in enumerate(obstacles_msg.array):
            bbox_center_x, bbox_center_y = self.calculate_bounding_box_center(obj.bev.data)
            _,dist = self.cal_obs_data(bbox_center_x,bbox_center_y)
            print("=====================")
            print("uturn  x,y   check")
            print("x,y",bbox_center_x,bbox_center_y)
            print("dist",dist)
            print("=====================")
            if(1<bbox_center_x<5 and -4<bbox_center_y<4):
                print("mid","(",bbox_center_x/2,bbox_center_y/2,")")
                self.left_obstacle=(bbox_center_x/2,bbox_center_y/2)

        self.publish_obstacles(self.left_obstacle, self.left_point_pub, color=(0.0, 0.0, 1.0))
    
    def calculate_bounding_box_center(self, bev_coords):
        center_x = (bev_coords[0] + bev_coords[2] + bev_coords[4] + bev_coords[6]) / 4
        center_y = (bev_coords[1] + bev_coords[3] + bev_coords[5] + bev_coords[7]) / 4
        return center_x, center_y
    
    def publish_obstacles(self, obstacle, publisher, color):
        if obstacle is not None:
            x, y = obstacle
            marker = Marker()
            marker.header.frame_id = "velodyne"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "obstacles"
            marker.id = 0
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.a = 1.0
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]

            publisher.publish(marker)
        
    def cal_obs_data(self,delta_x,delta_y):
        x=delta_x
        y=delta_y

        obs_angle = np.rad2deg(math.atan2(y,x))
        obs_dist = np.sqrt(x**2+y**2)

        return obs_angle, obs_dist

def main():
    rospy.init_node("lidar_obstacle_visualizer")
    visualizer = LidarObstacleVisualizer()
    rospy.spin()

if __name__ == "__main__":
    main()
