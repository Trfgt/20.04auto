#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from tracking_msg.msg import TrackingObjectArray

def tracking_object_callback(msg):
    for obj in msg.array:
        bbox2d = obj.bbox2d
        x1, y1, x2, y2 = bbox2d[0], bbox2d[1], bbox2d[2], bbox2d[3]

        bbox_width = x2 - x1
        bbox_height = y2 - y1

        bbox_center_x = (x1 + x2) / 2
        bbox_center_y = (y1 + y2) / 2

        print("Bounding Box Width:", bbox_width)
        print("Bounding Box Height:", bbox_height)
        print("Bounding Box Center X:", bbox_center_x)
        print("Bounding Box Center Y:", bbox_center_y)

def main():
    rospy.init_node('tracking_node')
    rospy.Subscriber('tracking_topic', TrackingObjectArray, tracking_object_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
