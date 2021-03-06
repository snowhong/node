import cv2 
import rospy
import os
import sys 
import roslib
import rospkg
import time
import std_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from skk.msg import UserTrackerPose
from skk.msg import UserTrackerPoseArray
from primesense import openni2
from primesense import nite2
import matplotlib.pyplot as plt
import numpy as np

class Skeleton():
    def __init__(self):
        nite2.initialize()
        self.ut = nite2.UserTracker.open_any()
        self.pub = rospy.Publisher('skeleton', UserTrackerPoseArray, queue_size=1)
        self.rate = rospy.Rate(1000)

        self.utpa = UserTrackerPoseArray()
        self.utp = UserTrackerPose()
        self.h = std_msgs.msg.Header()
        self.head_pose = Vector3()
        rate = rospy.Rate(100)


    def pose_transform(self,head):
        #Camera Parameters
        focal_length = 525.0
        optical_x = 319.5 
        optical_y = 239.5

        #format the raw pose data that can match with pixel in image
        #TODO: find a more accurate transform
        pixel_x = ((- head.position.x) * focal_length)/head.position.z
        pixel_y = ((- head.position.y) * focal_length)/head.position.z
        #set it in meter unit
        meter_z = head.position.z/1000.0
        
        pixel_center = Vector3()
        pixel_center.x = optical_x + pixel_x
        pixel_center.y = optical_y + pixel_y
        pixel_center.z = meter_z

        return pixel_center

    def skeleton_tracker(self):
        while not rospy.is_shutdown():
            frame = self.ut.read_frame()
            #depth_frame = frame.get_depth_frame()
            #depth_frame_data = depth_frame.get_buffer_as_uint16()
            #depth_array = np.ndarray((depth_frame.height,depth_frame.width),dtype=np.uint16,buffer=depth_frame_data)

            #Need to clear head pose data in the array
            self.utp.head=Vector3()
            self.utpa.users=[]
            active = 0
            dont_pub_blank_msg = 1#Now just let it publish blank message

            #Blank Message
            self.h.stamp = rospy.Time.now()
            self.utpa.header = self.h

            for fu in frame.users:
                user = fu
                if user.is_new():
                    self.ut.start_skeleton_tracking(fu.id)
                    print '=============================================================get new user:', fu.id
                #state 2(TRACKED STATES) means skeleton is active, if its a new user don't need to get state.
                elif user.skeleton.state == 2:
                    #head flag
                    head = user.skeleton.joints[0]
                    if(head.positionConfidence > 0.5):
                        print 'user id:', fu.id, 'head pose:', head.position
                        tmp = self.pose_transform(head)

                        self.head_pose.x = tmp.x
                        self.head_pose.y = tmp.y
                        self.head_pose.z = tmp.z
                        self.utp.uid = fu.id
                        self.utp.head = self.head_pose

                        self.h.stamp = rospy.Time.now()
                        self.utpa.header = self.h
                        self.utpa.users.append(self.utp)
                        active = active + 1
                        dont_pub_blank_msg = 1

                        
            #publish UserTrackerPoseArray(this must be out of the for loop to pub info that includes all user)
            if dont_pub_blank_msg:
                self.utpa.numUsers = active
                self.pub.publish(self.utpa)
                #print 'self.utpa:', self.utpa

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('skeleton_server')
    try:
        sk = Skeleton()
        sk.skeleton_tracker()
    except rospy.ROSInterruptException:
        pass
