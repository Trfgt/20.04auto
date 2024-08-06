#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
# from obstacle_detector.msg import Obstacles, CircleObstacle
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float32MultiArray
from tracking_msg.msg import TrackingObjectArray
from gps import GPS2UTM

class LidarObstacleVisualizer(GPS2UTM):
    def __init__(self):
        super().__init__()
        rospy.loginfo("LiDAR Receiver Object is Created")
        self.obstacles_sub = rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.obstacle_callback)
        # self.obstacles_sub = rospy.Subscriber("/obstacles", Obstacles, self.)
        # self.right_point_pub = rospy.Publisher("right_point", Marker, queue_size=1)
        self.left_point_pub = rospy.Publisher("left_point", Marker, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.obstacle_callback)  # 0.1 seconds interval, adjust as needed
        
        # self.middle_point_pub = rospy.Publisher("middle_point", Marker, queue_size=1)
        # self.target_point_publisher = rospy.Publisher("target_point", Float32MultiArray, queue_size=10)
        self.text_marker_pub = rospy.Publisher("obstacle_text_markers", MarkerArray, queue_size=10)
        

        
        # 속도 조정하며 바꿀 거리 범위 파라미터 앞방향이 x<0
        self.min_range = 1.5
        self.max_range = 5
        
        # first_right_obstacle과 가장 가까운 오른쪽 장애물들을 저장하는 변수
        self.right_obstacle = None
        # first_left_obstacle과 가장 가까운 왼쪽 장애물들을 저장하는 변수
        self.left_obstacle = None

    def obstacle_callback(self, obstacles_msg):
        # print("Orientation",self.Orientation)
        # print("Heading",self.heading)
        self.total_obj_cnt = obstacles_msg.size
        # 가장 먼저 찾은 오른쪽 좌표를 first_right_obstacle로 설정
        # if self.right_obstacle is None:
        #     self.right_obstacle = self.find_first_obstacle(obstacles_msg,'right_lane')
        # else:
        #  # right_obstacle과 가장 가까운 오른쪽 장애물들로 업데이트
        #     self.right_obstacle = self.find_right_closest_obstacles(obstacles_msg, self.right_obstacle)
        # self.publish_obstacles(self.right_obstacle, self.right_point_pub, color=(1.0, 0.0, 0.0))  # 빨간색으로 시각화
        # # rospy.loginfo(self.right_obstacle)
        
        # 가장 먼저 찾은 왼쪽 좌표를 first_left_obstacle로 설정
        if self.left_obstacle is None:
            self.left_obstacle = self.find_first_obstacle(obstacles_msg,'left_lane')

        # left_obstacle과 가장 가까운 왼쪽 장애물들로 업데이트
        else:
            self.left_obstacle = self.find_left_closest_obstacles(obstacles_msg, self.left_obstacle)
            if (self.left_obstacle is not None):
                self.delta_x = self.left_obstacle[0]
                self.delta_y = self.left_obstacle[1]
                left_utm_x, left_utm_y = self.calculate_longitude_latitude()
                
                self.publish_text_markers((left_utm_x, left_utm_y))

                self.publish_obstacles(self.left_obstacle, self.left_point_pub, color=(0.0, 0.0, 1.0))  # 파란색으로 시각화
    
        # middle_point = self.find_middle_point()
        # self.publish_obstacles(middle_point, self.middle_point_pub, color=(0.0, 1.0, 0.0))  # 초록색으로 시각화
        # Publish text markers for right, left, and middle points
        # self.publish_text_markers(middle_point, self.right_obstacle, self.left_obstacle)
        
        # self.delta_y = self.left_obstacle[1]
        # left_utm_x, left_utm_y = self.calculate_longitude_latitude()
        # if (self.left_obstacle is not None):
        #     self.delta_x = self.left_obstacle[0]
        #     self.delta_y = self.left_obstacle[1]
        #     left_utm_x, left_utm_y = self.calculate_longitude_latitude()
            
        #     self.publish_text_markers((left_utm_x, left_utm_y))
        

    def find_first_obstacle(self, obstacles_msg,direction):
        # center_y > 0 인 첫 번째 오른쪽 장애물을 설정
        min_distance_right = float('inf')
        min_distance_left = 3
        self.closest_obstacle = None
    
         # 패키지상에서 지정된 ROI내의 obj 개수 파악 
        for i in range(self.total_obj_cnt):
            center_x, center_y = obstacles_msg.array[i].point.x, obstacles_msg.array[i].point.y
            
            # if(-0.3<self.data.array[i].point.y < 0.3 and self.data.array[i].point.x < 5):
            #     rospy.loginfo("%.2f, %.2f, %.2f",  self.data.array[i].point.x, self.data.array[i].point.y, self.data.array[i].point.z)
                # static_range_cnt+=1
            distance = math.sqrt(center_x**2 + center_y**2)
            # print(center_y)

            # if(direction=='right_lane'):
            #     if self.min_range < center_x < self.max_range and center_y < 0 and distance < min_distance_right: #최솟값으로 업데이트? 
            #         self.closest_obstacle = (center_x, center_y)
            #         min_distance_right = distance
            if(direction=='left_lane'):
                if self.min_range < center_x < self.max_range and center_y < 0 and distance < min_distance_left:
                    self.closest_obstacle = (center_x, center_y)
                    min_distance_left = distance
                    if(center_x>4):
                        print("warning1111")
                    
        return self.closest_obstacle
    

    def find_left_closest_obstacles(self, obstacles_msg, reference_obstacle):
        # closest_obstacle = reference_obstacle # 상대좌표로 하니까 이상해짐 
        closest_obstacle = None
        min_distance = float('inf') # 초기화
        current_utm = (float('inf'),float('inf'))
        ref_x, ref_y = reference_obstacle 
        print("ref_x",ref_x)
        #---------utm 변환------------------------------------------
        self.delta_x = ref_x
        self.delta_y = ref_y
        ref_utm_x, ref_utm_y = self.calculate_longitude_latitude()
        print("ref_utm",ref_utm_x,ref_utm_y)
        #---------------utm -> distance 계산 

        for i in range(self.total_obj_cnt):
            center_x, center_y = obstacles_msg.array[i].point.x, obstacles_msg.array[i].point.y
            # self.delta_x = center_x
            # self.delta_y = center_y
            
            #---------utm 변환------------------------------------------
            self.delta_x = center_x
            self.delta_y = center_y
            # print("centerx",center_x)
            utm_x, utm_y = self.calculate_longitude_latitude()

            # print("----------",utm_x,utm_y)
            #---------------------------------------------------------
            
            distance1 = math.sqrt((center_x - ref_x)**2 + (center_y - ref_y)**2) # 상대좌표상 거리 -> 불필요해보임 
            distance2 = math.sqrt((utm_x - ref_utm_x)**2 + (utm_y - ref_utm_y)**2) #utm 기준 실제 
            # print("left distance",distance1)
            # print("left utm distance",distance2)
            
            # if self.max_range < center_x < self.min_range and cenㄴter_y <= 0:
            # distance < min_distance : utm 
            if(0.2<distance2 < 3):
                if self.min_range < center_x<3 and center_y <= 0 and center_x>ref_x and distance2<min_distance:
                        min_distance = distance2
                        # print(distance2)
                        current_utm = (utm_x, utm_y)
                        # closest_obstacle = (center_x, center_y)
                        closest_obstacle = (center_x, center_y)
                        # if(center_x>4):
                        #     print("warning!!!!!")
        
        closet_utm_x, closet_utm_y = current_utm
        if (closet_utm_x!=float("inf")):
            print("current_utm_",closet_utm_x,closet_utm_y) # 이전 값이랑 가장 가까운 다음 장애물 좌표
            print("left min distance",min_distance) # 이전 값과 업데이트된 장애물과의 거리 

        # if(min_distance!=float("inf")):
        if(closest_obstacle):
            print("return",closest_obstacle)
            return closest_obstacle
        else:
            print("return",closest_obstacle)
            return
    
    
    # def find_right_closest_obstacles(self, obstacles_msg, reference_obstacle):
    #     # closest_obstacle = reference_obstacle
    #     closest_obstacle = None
    #     min_distance = float('inf')
    #     ref_x, ref_y = reference_obstacle
        

    #     self.delta_x = ref_x
    #     self.delta_y = ref_y
    #     ref_utm_x, ref_utm_y = self.calculate_longitude_latitude()

    #     for i in range(self.total_obj_cnt):
    #         center_x, center_y = obstacles_msg.array[i].point.x, obstacles_msg.array[i].point.y
    #         self.delta_x = center_x
    #         self.delta_y = center_y
    #         utm_x, utm_y = self.calculate_longitude_latitude()
    #         # distance = math.sqrt((utm_x - ref_utm_x)**2 + (utm_y - ref_utm_y)**2)
    #         distance1 = math.sqrt((center_x - ref_x)**2 + (center_y - ref_y)**2) 
    #         distance2 = math.sqrt((utm_x - ref_utm_x)**2 + (utm_y - ref_utm_y)**2) #utm 
            
    #         # print("----------",utm_x,utm_y)
    #         # print("right distance",distance1)
    #         # print("right utm distance",distance2)
    #         # if self.max_range < center_x < self.min_range and center_y >= 0:
    #         if self.min_range < center_x and center_y <= 0 and  0.2 < distance2 < min_distance:
    #                 min_distance = distance2
    #                 # closest_obstacle = (center_x, center_y)
    #                 closest_obstacle = (center_x, center_y)
    #     # print("right min distance",min_distance)
    #     return closest_obstacle

    # def find_middle_point(self):
    #     if self.left_obstacle is not None and self.right_obstacle is not None:
    #         middle_x = (self.left_obstacle[0] + self.right_obstacle[0]) / 2.0
    #         middle_y = (self.left_obstacle[1] + self.right_obstacle[1]) / 2.0
            
    #         #------------제어에 넘기는 부분-----------------
    #         target_msg = Float32MultiArray()
    #         target_msg.data = [middle_x, middle_y]
            
    #         self.target_point_publisher.publish(target_msg)
            
    #         return (middle_x, middle_y)
    #     else:
    #         return None

#---------------------------rviz에 display--------------------------------------------
    def publish_text_markers(self,left_obstacle):
        marker_array = MarkerArray()
        markers = [
            # (middle_point, "Middle Point", (1.0, 1.0, 0.0)),
            # (right_obstacle, "Right Point", (1.0, 0.0, 0.0)),
            (left_obstacle, "Left Point", (0.0, 0.0, 1.0))
        ]
        print(left_obstacle)
        # for idx, (point, text, color) in enumerate(markers):
        #     if point is not None:
        #         x, y = point
        #         marker = Marker()
        #         marker.header.frame_id = "velodyne"
        #         marker.header.stamp = rospy.Time.now()
        #         marker.ns = "text_markers"
        #         marker.id = idx
        #         marker.type = Marker.TEXT_VIEW_FACING
        #         marker.action = Marker.ADD
        #         marker.pose.position.x = x
        #         marker.pose.position.y = y
        #         marker.pose.position.z = 0.5  # Height above the ground to display the text
        #         marker.scale.z = 0.3  # Text size
        #         marker.color.a = 1.0
        #         marker.color.r = 1
        #         marker.color.g = 0
        #         marker.color.b = 0
        #         marker.text = text + f": ({x:.2f}, {y:.2f})"  # Display the coordinates as text

        #         marker_array.markers.append(marker)
        # left_obstacle
        markers = [
            (left_obstacle, "Left Point", (0.0, 0.0, 1.0))
        ]
        for idx, (point, text, color) in enumerate(markers):
            if point is not None:
                x, y = point
                marker = Marker()
                marker.header.frame_id = "velodyne"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "text_markers"
                marker.id = idx
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.5  # Height above the ground to display the text
                marker.scale.z = 1  # Text size
                marker.color.a = 1.0
                marker.color.r = 1
                marker.color.g = 0
                marker.color.b = 0
                marker.text = text + f": ({x:.2f}, {y:.2f})"  # Display the coordinates as text

                marker_array.markers.append(marker)

        self.text_marker_pub.publish(marker_array)
        
    def publish_obstacles(self, obstacle, publisher, color):
        if obstacle is not None:
            x, y = obstacle
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

def main():
    rospy.init_node("lidar_obstacle_visualizer")
    visualizer = LidarObstacleVisualizer()
    # new_class2= GPS2UTM()
    rospy.spin()

if __name__ == "__main__":
    main()
