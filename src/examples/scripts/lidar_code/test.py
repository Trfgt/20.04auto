#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def create_marker(points, obstacles=None):
    marker_msg = Marker()
    marker_msg.header.frame_id = "map"  # 프레임 ID 설정 ("map"을 원하는 프레임으로 변경)
    marker_msg.type = Marker.LINE_STRIP
    marker_msg.action = Marker.ADD
    marker_msg.scale.x = 0.05  # 선 두께
    marker_msg.color.a = 1.0  # 투명도
    marker_msg.color.r = 1.0  # 빨간색
    marker_msg.color.g = 0.0  # 녹색
    marker_msg.color.b = 0.0  # 파란색
    marker_msg.id = 0  # 마커 ID

    # Scale factor to adjust coordinates for better visualization
    scale_factor = 0.7
    offset_x = 20
    offset_y = 1100

    # Append four points to create a rectangle
    for i in range(4):
        p = Point()
        p.x = points[i][0] * scale_factor - offset_x
        p.y = points[i][1] * scale_factor - offset_y
        p.z = 0.0  # 모든 점의 z 좌표는 0으로 가정
        marker_msg.points.append(p)

    # Close the loop to form a rectangle
    p = Point()
    p.x = points[0][0] * scale_factor - offset_x
    p.y = points[0][1] * scale_factor - offset_y
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

    if obstacles is not None:
        # 장애물을 추가하여 시각화
        marker_msg.type = Marker.LINE_LIST
        marker_msg.color.a = 1.0  # 투명도
        marker_msg.color.r = 0.0  # 빨간색
        marker_msg.color.g = 0.0  # 녹색
        marker_msg.color.b = 1.0  # 파란색
        for obs in obstacles:
            obs_p1 = Point()
            obs_p1.x = obs[0] * scale_factor - offset_x
            obs_p1.y = obs[1] * scale_factor - offset_y
            obs_p1.z = 0.0
            obs_p2 = Point()
            obs_p2.x = obs[2] * scale_factor - offset_x
            obs_p2.y = obs[3] * scale_factor - offset_y
            obs_p2.z = 0.0
            marker_msg.points.append(obs_p1)
            marker_msg.points.append(obs_p2)

    return marker_msg

if __name__ == '__main__':
    try:
        rospy.init_node('rectangle_marker_publisher', anonymous=True)
        rospy.loginfo("node_start")
        pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

        parking_lots = [
            [(19.44, 1104.7), (23.55, 1102.27), (22.41, 1100.31), (18.42, 1102.55)],
            [(18.42, 1102.55), (22.41, 1100.31), (21.21, 1098.14), (17.12, 1100.4)],
            [(17.12, 1100.4), (21.21, 1098.14), (19.96, 1095.76), (16, 1098.23)],
            [(16, 1098.23), (19.96, 1095.76), (18.82, 1093.87), (14.67, 1095.94)]
        ]

        # 장애물 좌표 (예시)
        obstacles = [
            (18.0, 1100.5, 19.0, 1101.0),
            (21.0, 1099.0, 22.0, 1100.0)
        ]

        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            marker_array = MarkerArray()  # marker_array를 루프 안에서 초기화
            for i, points in enumerate(parking_lots):
                marker = create_marker(points, obstacles)
                marker.id = i  # 각각의 마커마다 고유한 ID를 설정
                marker_array.markers.append(marker)
            pub.publish(marker_array)
            rate.sleep()
       
    except rospy.ROSInterruptException:
        pass
