#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import os
from sensor_msgs.msg import Imu
# from morai_msgs.msg import GPSMessage
from std_msgs.msg import Int32, Float32MultiArray
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from pyproj import Proj
from tracking_msg.msg import TrackingObjectArray
import pyproj
from math import pi,sqrt
import math
from geometry_msgs.msg import Quaternion
from pyproj import CRS, Transformer
from geometry_msgs.msg import PoseArray, Pose
from custom_msg.msg import PointArray_msg # geometry_msgs/Point[] array -> custom msg 생성
import numpy as np
import time
from microstrain_inertial_msgs.msg import FilterHeading
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import Point, TwistStamped


class GPS2UTM:
    def __init__(self):
        rospy.loginfo("GPS2UTM is Created")
        # rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        # rospy.Subscriber("/imu", Imu, self.imu_callback)

        rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        rospy.Subscriber("odom_gps", Odometry, self.odom_callback)
        self.bev_pub = rospy.Publisher('bev', PoseArray, queue_size=10)
        self.parking_idx_pub = rospy.Publisher('parking_idx', Int32, queue_size=10)
        self.marker_pub2 = rospy.Publisher('line_marker', Marker, queue_size=10)
        self.marker_pub = rospy.Publisher('cylinder_markers', MarkerArray, queue_size=20)
        self.marker_pub1 = rospy.Publisher('barrier', MarkerArray, queue_size=20)
        # self.marker_pub2 = rospy.Publisher('obs_check', MarkerArveay, queue_size=50)

        #----------------변수 초기화---------------------
        self.temp1=self.temp2= self.tempa=self.tempb=0
        self.wheelbase=0
        self.is_gps=False
        self.current_time=0
        self.a_cnt=0;self.b_cnt=0;self.c_cnt=0
        self.trigger=False
        self.is_odom=False
        self.current_postion=Point()
        self.heading=0.
        #----------------------------------------------

    # def gps_callback(self,_data):
    #     self.start_longitude = _data.longitude
    #     self.start_latitude = _data.latitude
    #     self.e_o = _data.eastOffset
    #     self.n_o = _data.northOffset
    #     self.is_gps=True
    #     pass

    # def imu_callback(self, _data):
    #     self.Orientation = _data.orientation
    #     self.heading = self.calculate_heading_from_imu()
    #     # print(f"Heading: {self.heading}")
        # pass
    

    def odom_callback(self,msg):
        self.is_odom=True
        self.start_longitude=msg.pose.pose.position.x
        self.start_latitude=msg.pose.pose.position.y
        self.heading=msg.pose.pose.position.z

    def lidar_callback(self, _data):
        #ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
        spot_a=[326912.218,4127872.218]
        spot_d=[326930.1569,4127872.677]

        line_marker = self.create_line_marker(spot_a, spot_d)
        self.marker_pub2.publish(line_marker)
        #ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ

        # 초기화해줘야 함  -> 주차장 근처일때의 트리거 나중에 세팅
        self.a_cnt=0;self.b_cnt=0;self.c_cnt=0
        total_obj_cnt = _data.size

        # UTM 좌표들을 저장할 빈 리스트 생성
        pointcloud = []
        east_offset = 302459.942
        north_offset = 4122635.537
        # print("총 obs 개수",total_obj_cnt)

        # print(1/(time.time()-self.current_time))
        self.current_time=time.time()
        bev_msg = PoseArray()
        bev_msg.header = _data.header

        #---장애물 확인----------
        objects= MarkerArray()
        for i, obj in enumerate(_data.array):
            if len(obj.bev.data) < 8:
                rospy.logwarn("Invalid bounding box format: %s", obj.bev.data)
                continue
            # exception
            bbox_center_x, bbox_center_y = self.calculate_bounding_box_center(obj.bev.data)
            bbox_width, bbox_height = self.calculate_bounding_box_dimensions(obj.bev.data)
            self.delta_x = bbox_center_x + self.wheelbase
            self.delta_y = bbox_center_y
            # self.delta는 라이다와 gps 위치 차이 고려
            # wheelbase는 약 1.1로 설정. 부착위치에 따라 변경 요구
            x=bbox_center_x
            y=bbox_center_y
            
            # 라이다 고유 데이터
            obs_angle,obs_dist=self.cal_obs_data(x,y)
            # angle이랑 dist 계산
            # angle은 현재 degree 상태. -> cal_obs_data에 수정 가능.
            end_utm_easting, end_utm_northing = self.calculate_longitude_latitude()
            # if(i<2):
            print("i")
            print("==========================================")
            print('x:',end_utm_easting)
            print('y:',end_utm_northing)
            print("==========================================")
            # #------------장애물 utm 계산 맞는지 rviz 시각화로 확인
            # barrier_marker = Marker()
            # barrier_marker.header.frame_id = "map"
            # barrier_marker.header.stamp = rospy.Time.now()
            # barrier_marker.ns = "barrier"
            # barrier_marker.id = i
            # barrier_marker.type = Marker.CYLINDER
            # barrier_marker.action = Marker.ADD
            # barrier_marker.pose.position.x = end_utm_easting-east_offset
            # barrier_marker.pose.position.y = end_utm_northing-north_offset
            # barrier_marker.pose.position.z = 0.0
            # barrier_marker.scale.x = 0.3  # Diameter of the cylinder
            # barrier_marker.scale.y = 0.3  # Diameter of the cylinder
            # barrier_marker.scale.z = 2.0  # Height of the cylinder
            # barrier_marker.color.a = 1  # Transparency
            # barrier_marker.color.r = 1.0
            # barrier_marker.color.g = 0.0
            # barrier_marker.color.b = 0.0
            # objects.markers.append(barrier_marker)

            #----------주차장 판단----------------
            # if(parking_zone==True):
            self.count_obs_from_line(end_utm_easting,end_utm_northing) # self.a_cnt,, 업데이트
            marker_array1 = MarkerArray()

            if(self.trigger==True):

                print('iiiiiiiiiiiiii')
                print(i)
                barrier_marker = Marker()
                barrier_marker.header.frame_id = "map"
                barrier_marker.header.stamp = rospy.Time.now()
                barrier_marker.ns = "barrier"
                barrier_marker.id = i
                barrier_marker.type = Marker.CYLINDER
                barrier_marker.action = Marker.ADD
                barrier_marker.pose.position.x = end_utm_easting
                barrier_marker.pose.position.y = end_utm_northing
                barrier_marker.pose.position.z = 0.0
                barrier_marker.scale.x = 0.3  # Diameter of the cylinder
                barrier_marker.scale.y = 0.3  # Diameter of the cylinder
                barrier_marker.scale.z = 2.0  # Height of the cylinder
                barrier_marker.color.a = 1  # Transparency
                barrier_marker.color.r = 0.0
                barrier_marker.color.g = 0.0
                barrier_marker.color.b = 1.5
                marker_array1.markers.append(barrier_marker)

            # print("obs_x: {:.6f}, obs_y: {:.6f}".format(end_utm_easting - east_offset,end_utm_northing - north_offset))
             # UTM 좌표를 Point 메시지로 변환하여 리스트에 추가

            if obs_dist <= 5:
                # print("=========barrier===obs_data==================")
                # print("obs_x: {:.6f}, obs_y: {:.6f}".format(end_utm_easting - east_offset,end_utm_northing - north_offset))
               # print("obs_angle: {:.3f}, obs_dist: {:.3f}".format(obs_angle,obs_dist))
                # print("obs_width: {:.3f}, obs_height: {:.3f}".format(bbox_width,bbox_height))
                bev_pose = Pose()
                bev_pose.position.x = bbox_center_x
                bev_pose.position.y = bbox_center_y
                bev_pose.position.z = 0.0
                bev_pose.orientation.x = bbox_width
                bev_pose.orientation.y = bbox_height
                bev_pose.orientation.z = obs_dist
                bev_pose.orientation.w = obs_angle
                bev_msg.poses.append(bev_pose)

            # dist <= 5이하일 때만 장애물로 추가.
        if(self.a_cnt==0):
            print("AAAAAAAAAAAAAAAAAAAA")
            print("AAAAAAAAAAAAAAAAAAAA")
            print("AAAAAAAAAAAAAAAAAAAA")
            print("AAAAAAAAAAAAAAAAAAAA")

        elif(self.b_cnt==0):
            print("bbbbbbbbbbbbbbbbbbbbb")
            print("bbbbbbbbbbbbbbbbbbbbb")
            print("bbbbbbbbbbbbbbbbbbbbb")
            print("bbbbbbbbbbbbbbbbbbbbb")

        # el(self.c_cnt==0):
        #     print("ccccccccccccccccccccc")
        #     print("ccccccccccccccccccccc")
        #     print("ccccccccccccccccccccc")
        #     print("cccccccccccccbarriercccccccc")

        # if(parking_zone==True):  
        msg = Int32()
        msg.data = self.select_parking_lot()

        #---------------------------------
        # self.marker_pub2.publish(objects)
        #-----------------------------------
        self.parking_idx_pub.publish(msg)
            # rospy.sleep(1) # 1초마다 하나씩 발행
        # custom_msg/PointArray_msg.msg 메시지 생성
        self.bev_pub.publish(bev_msg)
        print(marker_array1)
        self.marker_pub1.publish(marker_array1)

            # # UTM 좌표를 리스트에 추가
            # pointcloud.append([end_utm_easting, end_utm_northing])

        # 리스트를 NumPy 2차원 배열barrier로 변환
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


    def create_line_marker(self, p1, p2):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.r = 1.0
        marker.color.a = 1.0
        point1 = Point()
        point1.x = p1[0]
        point1.y = p1[1]
        point1.z = 0.0
        point2 = Point()
        point2.x = p2[0]
        point2.y = p2[1]
        point2.z = 0.0
        marker.points.append(point1)
        marker.points.append(point2)
        return marker

    def cal_obs_data(self,delta_x,delta_y):
        x=delta_x
        y=delta_y
        obs_angle = np.rad2deg(math.atan2(y,x))
        obs_dist = np.sqrt(x**2+y**2)
        return obs_angle, obs_dist

    def calculate_longitude_latitude(self):
        # 시작 위치의 UTM 좌표 구하기
        # transformer = Transformer.from_crs(CRS.from_epsg(4326), CRS.from_epsg(32652), always_xy=True)  # UTM Zone 52
        # start_utm_easting, start_utm_northing = transformer.transform(self.start_longitude, self.start_latitude)
        # print(self.start_longitude, self.start_latitude)
        # # 차량 UTM 좌표
        # print("Start UTM Easting:",start_utm_easting, "Start UTM Northing:", start_utm_northing)
        # 프레임별 차량 UTM좌표 차이
        # print("----start", start_utm_easting-self.tempa, start_utm_northing-self.tempb)

        # 헤딩값을 라디안 단위로 변환
        heading_rad = math.radians(self.heading)
        # heading_rad = math.radians(self.vehicle_yaw)
        # print("heading_rad", heading_rad*180/pi)

        # self.tempa= start_utm_easting
        # self.tempb= start_utm_northing

        # 상대적인 x와 y 위치를 UTM 좌표계로 변환
        delta_utm_easting = self.delta_x * math.cos(heading_rad) - self.delta_y * math.sin(heading_rad)
        delta_utm_northing = self.delta_x * math.sin(heading_rad) + self.delta_y * math.cos(heading_rad)
        # print(heading_rad)
        # obs_angle=(math.atan2(delta_utm_northing,delta_utm_easting)-heading_rad+2*pi)%(2*pi)
        dist=sqrt(delta_utm_easting**2+delta_utm_northing**2)
        # print('dist:',dist)
        # if obs_angle > pi:
        #     obs_angle-=2*pi
        # if abs(obs_angle)*180/pi<20 and dist < 5:
        #     print('dist:',dist)
        # print("obs_angle",obs_angle*180/pi)
        # 부호가 반대일 수도 있음.
        # 차량의 방향을 기준으로 장애물이 어디에 위치돼 있는지

        # 시작 위치에 UTM 좌표 변화량을 더하여 최종 UTM 좌표 구하기
        end_utm_easting = self.start_longitude + delta_utm_easting
        end_utm_northing = self.start_latitude + delta_utm_northing

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

    def distance_from_line(self,x, y, m=-227805.195, b=74507126688.5):
        numerator = abs(m * x - y + b)
        denominator = math.sqrt(m ** 2 + 1)
        return numerator / denominator

    def count_obs_from_line(self,x,y):
        dist = self.distance_from_line(x,y)
        # print("=====dist from line=====")
        # print('dist',dist)
        # print('------------------------')
        # spot_a=[20.22, 1127.04]
        # spot_b=[17.78, 1122.67]
        # spot_c=[15.38, 1118.3]
        # spot_d=[12.26, 1112.65]

        spot_a=[326912.218,4127872.218]

        spot_b=[326918.224,4127872.333]

        spot_c=[326923.990,4127872.413]

        spot_d=[326930.1569,4127872.677]

        # spot_e=[15.16,1117.35]

        # spot_f=[13.94,1114.98]

        marker_array = MarkerArray()


        spots = [spot_a,spot_b,spot_c,spot_d]

        self.trigger=False

        for idx, val in enumerate(spots):
            cylinder_marker = Marker()
            cylinder_marker.header.frame_id = "map"
            cylinder_marker.header.stamp = rospy.Time.now()
            cylinder_marker.ns = "parking_spots"
            cylinder_marker.id = idx
            cylinder_marker.type = Marker.CYLINDER
            cylinder_marker.action = Marker.ADD
            cylinder_marker.pose.position.x = val[0]
            cylinder_marker.pose.position.y = val[1]
            cylinder_marker.pose.position.z = 0.0
            cylinder_marker.scale.x = 0.3  # Diameter of the cylinder
            cylinder_marker.scale.y = 0.3  # Diameter of the cylinder
            cylinder_marker.scale.z = 2.0  # Height of the cylinder
            cylinder_marker.color.a = 0.5  # Transparency
            cylinder_marker.color.r = 1.0
            cylinder_marker.color.g = 0.0
            cylinder_marker.color.b = 0.0
            marker_array.markers.append(cylinder_marker)
        print(cylinder_marker)
        if(dist<1):

            print("!!!!!!!!!!!!!!closet!!!!!!!!!!!!")

            print("x",x,"y",y)

            # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

            # 범위 설정 모라이로 검증해보기 좌표계 방향에 따라 범위의 직사각형이 달라짐

            if(spot_b[0]<x<spot_a[0] and spot_b[1]<y<spot_a[1]):

                print("111111")

                self.a_cnt+=1

                self.trigger=True

            if(spot_d[0]<x<spot_c[0] and spot_d[1]<y<spot_c[1]):

                print("222222")

                self.b_cnt+=1

                self.trigger=True

            else:

                print("333333")

                self.c_cnt+=1

                self.trigger=True

        else:
            pass
        
        self.marker_pub.publish(marker_array)

        return

 

    def select_parking_lot(self):

        parking_lot=[self.a_cnt,self.b_cnt,self.c_cnt]

        print("cnt")

        print(parking_lot)

        select=None

        for idx, val in enumerate(parking_lot):

            print("idx: {idx}, val: {val}")

            if(val<1):

                select=idx

        if(select is not None):

            return select

        else:

            return

   

 

 

def run():

    rospy.init_node("gps2utm")

    new_classs= GPS2UTM()

    rospy.spin()

   

 

if __name__ == '__main__':

    try:

        run()

    except rospy.ROSInterruptException:

        pass