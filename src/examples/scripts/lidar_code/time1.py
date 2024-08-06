#! /usr/bin/env python2
# -*- coding: utf-8 -*-

import time
import math
import tf.transformations as tft
from scipy.spatial.transform import Rotation as R


quaternion = [0.1, 0.2, 0.3, 0.4]

# tf.transformations 벤치마킹
start = time.time()
for _ in range(100000):
    euler = tft.euler_from_quaternion(quaternion)
end = time.time()
print("tf.transformations: ", end - start)

# scipy 벤치마킹
start1 = time.time()
for _ in range(100000):
    r = R.from_quat(quaternion)
    euler = r.as_euler('zyx', degrees=False)
end1 = time.time()
print("scipy: ", end1 - start1)

class QuaternionToEuler:
    def quaternion_to_euler(self, x, y, z, w):
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

converter = QuaternionToEuler()

quaternion = [0.1, 0.2, 0.3, 0.4]

# 주어진 함수의 벤치마킹
start2 = time.time()
for _ in range(100000):
    euler = converter.quaternion_to_euler(*quaternion)
end2 = time.time()
print("Custom function: ", end2 - start2)