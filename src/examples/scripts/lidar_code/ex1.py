#!/usr/bin/env python3
#-*- coding: utf-8 -*-
import rospy
from tracking_msg.msg import TrackingObjectArray
from sensor_msgs.msg import Imu
from autosense_msgs.msg import TrackingFixedTrajectoryArray
# rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

class LidarReceiver():
    def __init__(self):
        rospy.loginfo("LiDAR Receiver Object is Created")
        rospy.Subscriber("/lidar/tracking_objects", TrackingObjectArray, self.lidar_callback)
        rospy.Subscriber("imu",Imu, self.Imu_callback)
        rospy.Subscriber("/tracking/fixed_trajectories", TrackingFixedTrajectoryArray, self.track_callback) # 아직 사용 안 함 
        
    def track_callback(self, _data): # 
        # rospy.loginfo (_data)
        pass
    
    def Imu_callback(self, _data):
        # rospy.loginfo("linear_acc %.2f",_data.linear_acceleration.y)
        # self.y_vel = _data.orientation
        # rospy.loginfo("1, %.2f",self.y_vel)
        # rospy.loginfo(self.y_vel
        pass
        
        
    def lidar_callback(self, _data):
        detect = Detect(_data)
        detect.static()
        
        
class Detect(): # 인지
    def __init__(self,data):
        self.data = data
        # self.y_vel1=y_vel

    def static(self):
        # rospy.loginfo(self.data)
        # rospy.loginfo(self.y_vel1)
        total_obj_cnt = self.data.size # 패키지상에서 지정된 ROI내의 obj 개수 파악 
        static_range_cnt=0
        # for i in range(total_obj_cnt):
        #     rospy.loginfo("2, %.2f", self.data.array[i].velocity.y)
            
        for i in range(total_obj_cnt):
            # rospy.loginfo(self.data.array[i].velocity.y)
            # rospy.loginfo(self.data.array[i].inpath)
            # rospy.loginfo("id:"+str(i))
            # rospy.loginfo("%.2f, %.2f, %.2f",  self.data.array[i].point.x, self.data.array[i].point.y, self.data.array[i].point.z)
            
            if(-0.3<self.data.array[i].point.y < 0.3 and self.data.array[i].point.x < 5):
                rospy.loginfo("%.2f, %.2f, %.2f",  self.data.array[i].point.x, self.data.array[i].point.y, self.data.array[i].point.z)
                static_range_cnt+=1
            
        rospy.loginfo("static_cnt= %d", static_range_cnt)
    
    def dynamic(self):
        pass
    
    def Labacon(self):
        pass

class Labacon(Detect): 
    pass
        

def run():
    rospy.init_node("lidar")
    new_class= LidarReceiver()
    rospy.spin()
    

if __name__ == '__main__':
    run()