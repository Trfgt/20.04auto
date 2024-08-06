#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64

class LinearVelocityEstimator:
    def __init__(self):
        rospy.init_node('linear_velocity_estimator')
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.linear_velocity_pub = rospy.Publisher('/linear_velocity', Vector3, queue_size=10)
        self.prev_timestamp = None
        self.prev_linear_acceleration = None
        self.linear_velocity = Vector3()

    def imu_callback(self, imu_msg):
        current_timestamp = imu_msg.header.stamp.to_sec()

        if self.prev_timestamp is not None:
            dt = current_timestamp - self.prev_timestamp

            # Linear acceleration
            linear_acceleration = imu_msg.linear_acceleration

            if self.prev_linear_acceleration is not None:
                # Estimate linear velocity using trapezoidal integration
                delta_linear_acceleration = Vector3(
                    (linear_acceleration.x + self.prev_linear_acceleration.x) * 0.5 * dt,
                    (linear_acceleration.y + self.prev_linear_acceleration.y) * 0.5 * dt,
                    (linear_acceleration.z + self.prev_linear_acceleration.z) * 0.5 * dt
                )
                self.linear_velocity.x += delta_linear_acceleration.x
                self.linear_velocity.y += delta_linear_acceleration.y
                self.linear_velocity.z += delta_linear_acceleration.z

            # Publish the estimated linear velocity
            self.linear_velocity_pub.publish(self.linear_velocity)

            self.prev_linear_acceleration = linear_acceleration

        self.prev_timestamp = current_timestamp

def main():
    linear_velocity_estimator = LinearVelocityEstimator()
    rospy.spin()

if __name__ == '__main__':
    main()

