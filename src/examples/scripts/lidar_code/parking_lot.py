#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def create_marker(points):
    marker_msg = Marker()
    marker_msg.header.frame_id = "map"  # 프레임 ID 설정 ("map"을 원하는 프레임으로 변경)
    marker_msg.type = Marker.LINE_STRIP
    marker_msg.action = Marker.ADD
    marker_msg.scale.x = 0.15  # 선 두께
    marker_msg.color.a = 1.0  # 투명도
    marker_msg.color.r = 0.0  # 빨간색
    marker_msg.color.g = 1.0  # 녹색
    marker_msg.color.b = 0.0  # 파란색
    marker_msg.id = 0  # 마커 ID
    
    # Scale factor to adjust coordinates for better visualization
    scale_factor = 0.7
    # offset_x = 20
    # offset_y = 784
    east_offset = 302459.942
    north_offset = 4122635.537

    # Append four points to create a rectangle
    for i in range(4):
        p = Point()
        p.x = points[i][0] + east_offset
        p.y = points[i][1] + north_offset
        p.z = 0.0  # 모든 점의 z 좌표는 0으로 가정
        marker_msg.points.append(p)

    # Close the loop to form a rectangle
    p = Point()
    p.x = points[0][0] + east_offset
    p.y = points[0][1] + north_offset
    p.z = 0.0
    marker_msg.points.append(p)

    # Marker 옵션 설정 (COLLISION_OBJECT 활성화)
    marker_msg.lifetime = rospy.Duration(0)  # 영구적인 마커로 설정
    marker_msg.action = Marker.ADD
    marker_msg.frame_locked = True
    marker_msg.ns = "parking_lots"
    marker_msg.pose.orientation.w = 1.0
    marker_msg.pose.orientation.x = 0.0
    marker_msg.pose.orientation.y = 0.0
    marker_msg.pose.orientation.z = 0.0

    return marker_msg

if __name__ == '__main__':
    try:
        rospy.init_node('rectangle_marker_publisher', anonymous=True)
        pub = rospy.Publisher('parking_lotssss', MarkerArray, queue_size=10)

        parking_lots = [
            # [(29.8, 1124.2), (34.61, 1124.54), (34.25, 1121.91), (29.87, 1121.87)],
            # [(28.39, 1121.57), (33.14, 1121.78), (33.06, 1119.39), (28.42, 1119.24)],
            # [(26.99, 1119.06), (31.65, 1119.17), (31.54, 1116.86), (27.07, 1116.74)],
            # [(25.61, 1116.35), (30.04, 1116.34), (30.15, 1114.3), (25.66, 1114.09)],
            # [(24.26, 1113.75), (28.66, 1113.86), (28.71, 1111.76), (24.29, 1111.59)],
            # [(22.7, 1111.15), (27.26, 1111.35), (27.31, 1109.3), (22.84, 1108.95)]
            # [(10.98, 1089.91), (15.618, 1091.67), (16.31, 1089.32), (11.65, 1087.61)],
            # [(9.07, 1086.64), (13.75, 1088.24), (14.54, 1086.03), (9.81, 1084.35)],
            # [(7.2, 1083.23), (11.88, 1084.94), (12.6, 1082.65), (7.96, 1080.9)],
            # # 주차장 추가
            # [(-4.55, 1088.35), (-2.15, 1092.7), (0.34, 1091.34), (-2.09, 1086.95)],
            # [(-2.14, 1092.73), (0.29, 1097.09), (2.78, 1095.72), (0.36, 1091.33)],

            # [(-2.14, 1092.73), (0.29, 1097.09), (2.78, 1095.72), (0.36, 1091.33)],
            [(3.39, 1102.68), (9.71, 1114.09), (12.26, 1112.65), (5.89, 1101.29)],
            [(12.84, 1119.69), (15.3, 1124.11), (17.79, 1122.65), (15.38, 1118.3)],
            [(15.3, 1124.13), (17.76, 1128.54), (20.22, 1127.04), (17.78, 1122.67)],

            302476.639569, 4123755.660857  302474.301836, 4123751.362320 # 3

            302474.301836, 4123751.362320  302479.112845, 4123760.190940 # 2
            

            302476.639569, 4123755.660857 302474.301836, 4123751.362320

            302479 .112845, 4123760.190940  302476.639569, 4123755.660857

            302479 .112845, 4123760.190940  302481.226199, 4123763.980483 # 1 

            # ,4123763.980483,114
            #     self.add_marker(_data.header,"map",302476.639569,4123755.660857, 111)
            #     self.add_marker(_data.header,"map",302474.301836,4123751.362320,112)
            #     self.add_marker(_data.header,"map",302479.112845,4123760.190940,113)
        
            
        ]

        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            marker_array = MarkerArray()  # marker_array를 루프 안에서 초기화
            for i, points in enumerate(parking_lots):
                marker = create_marker(points)
                marker.id = i  # 각각의 마커마다 고유한 ID를 설정
                marker_array.markers.append(marker)
            pub.publish(marker_array)
            rate.sleep()
       
    except rospy.ROSInterruptException:
        pass
