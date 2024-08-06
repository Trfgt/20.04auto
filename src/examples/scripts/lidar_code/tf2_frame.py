#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage
from geographic_msgs.msg import GeoPoint
from pyproj import Proj, transform

# GPS와 IMU 데이터를 저장할 변수
gps_data = None
imu_data = None

# WGS 84 (latitude, longitude) 좌표계와 UTM 좌표계 사이의 변환을 정의
WGS84 = Proj(init='epsg:4326') # WGS 84
UTM = Proj(init='epsg:32632') # UTM zone 32N

def convert_gps_to_local(lat, lon):
    x, y = transform(WGS84, UTM, lon, lat) # longitude, latitude 순서에 주의
    return x, y

def gps_callback(data):
    global gps_data
    gps_data = data

def imu_callback(data):
    global imu_data
    imu_data = data

def main():
    rospy.init_node('tf2_broadcaster')

    # GPS와 IMU 데이터를 구독
    rospy.Subscriber('/gps', GPSMessage, gps_callback)
    rospy.Subscriber('/imu', Imu, imu_callback)

    br = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(10.0)  # 10Hz로 변환을 게시
    while not rospy.is_shutdown():
        try:
            # 두 센서 데이터가 모두 있다면 변환을 생성 및 게시
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "/map"
            t.child_frame_id = "/velodyne"

            # GPS 데이터를 메터 단위의 로컬 좌표로 변환
            t.transform.translation.x, t.transform.translation.y = convert_gps_to_local(gps_data.latitude, gps_data.longitude)

            # IMU 데이터를 쿼터니언으로 변환
            t.transform.rotation = imu_data.orientation

            br.sendTransform(t)
        except:
            pass

        rate.sleep()

if __name__ == '__main__':
    main()
