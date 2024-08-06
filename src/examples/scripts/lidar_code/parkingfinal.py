#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

def create_marker(points):
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

    scale_factor = 0.7
    offset_x = 20
    offset_y = 1100

    for i in range(4):
        p = Point()
        p.x = points[i][0] - offset_x
        p.y = points[i][1] - offset_y
        p.z = 0.0
        marker_msg.points.append(p)

    p = Point()
    p.x = points[0][0] - offset_x
    p.y = points[0][1] - offset_y
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

def create_obstacle_marker(point):
    marker_msg = Marker()
    marker_msg.header.frame_id = "map"
    marker_msg.type = Marker.SPHERE
    marker_msg.action = Marker.ADD
    marker_msg.scale.x = 0.5
    marker_msg.scale.y = 0.5
    marker_msg.scale.z = 0.5
    marker_msg.color.a = 1.0
    marker_msg.color.r = 1.0  # 장애물 색상을 빨간색으로 설정
    marker_msg.color.g = 0.0
    marker_msg.color.b = 0.0
    marker_msg.pose.orientation.w = 1.0
    marker_msg.pose.position.x = point[0] - 20  # offset_x 값 적용
    marker_msg.pose.position.y = point[1] - 1100  # offset_y 값 적용

    return marker_msg

def is_point_inside_polygon(points, point):
    n = len(points)
    inside = False
    p1x, p1y = points[0]
    for i in range(n + 1):
        p2x, p2y = points[i % n]
        if point[1] > min(p1y, p2y):
            if point[1] <= max(p1y, p2y):
                if point[0] <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (point[1] - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or point[0] <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

if __name__ == '__main__':
    try:
        rospy.init_node('rectangle_marker_publisher', anonymous=True)
        rospy.loginfo("노드 시작")
        pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

        parking_lots = [
            [(19.44, 1104.7), (23.55, 1102.27), (22.41, 1100.31), (18.42, 1102.55)],
            [(18.42, 1102.55), (22.41, 1100.31), (21.21, 1098.14), (17.12, 1100.4)],
            [(17.12, 1100.4), (21.21, 1098.14), (19.96, 1095.76), (16, 1098.23)],
            [(16, 1098.23), (19.96, 1095.76), (18.82, 1093.87), (14.67, 1095.94)]
        ]

        # 가상의 장애물의 좌표 설정s
        obstacle_point = (18.42, 1095.71)

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            marker_array = MarkerArray()
            for i, points in enumerate(parking_lots):
                marker = create_marker(points)
                marker.id = i

                # 사각형 내부에 장애물이 있는 경우 해당 사각형만 빨간색으로 표시
                is_inside_obstacle = is_point_inside_polygon(points, obstacle_point)
                if is_inside_obstacle:
                    marker.color.r = 1.0
                    marker.color.g = 0.0
                else:
                    marker.color.r = 0.0
                    marker.color.g = 1.0

                marker_array.markers.append(marker)

            # 장애물 마커 추가 (빨간색으로 표시)
            obstacle_marker = create_obstacle_marker(obstacle_point)
            marker_array.markers.append(obstacle_marker)

            pub.publish(marker_array)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass