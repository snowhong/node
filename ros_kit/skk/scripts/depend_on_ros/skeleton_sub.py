import cv2 
import rospy
import os
import sys 
import roslib
import rospkg
import time
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
#from skk.msg import UserTracker
#from skk.msg import UserTrackerArray
from skk.msg import UserTrackerPose
from skk.msg import UserTrackerPoseArray
from primesense import openni2
from primesense import nite2
import matplotlib.pyplot as plt

class Skeleton():
    def __init__(self):
        nite2.initialize()
        self.ut = nite2.UserTracker.open_any()
        self.pub = rospy.Publisher('skeleton', UserTrackerPoseArray, queue_size=10)
        self.rate = rospy.Rate(10)

        self.utpa = UserTrackerPoseArray()
        self.utp = UserTrackerPose()
        self.head_pose = Vector3()
        rate = rospy.Rate(100)

    def skeleton_tracker(self):
        while not rospy.is_shutdown():
            frame = self.ut.read_frame()
            depth_frame = frame.get_depth_frame()
            depth_frame_data = depth_frame.get_buffer_as_uint16()
            depth_array = np.ndarray((depth_frame.height,depth_frame.width),dtype=np.uint16,buffer=depth_frame_data)

            #Need to clear head pose data in the array
            self.utp.projective=[]
            self.utpa.users=[]
            for fu in frame.users:
                user = fu
                if user.is_new():
                    self.ut.start_skeleton_tracking(fu.id)
                    print 'get new user:', fu.id
                    #print 'user skeleton state:',user.skeleton.state
                # 2 means skeleton ok, if its a new user don't need to get state.
                elif user.skeleton.state == 2:
                    #head flag
                    head = user.skeleton.joints[0]
                    if(head.positionConfidence > 0.5):
                        print 'user id:', fu.id, 'head pose:', head.position
                        self.head_pose.x = head.position.x
                        self.head_pose.y = head.position.y
                        self.head_pose.z = head.position.z
                        self.utp.uid = fu.id
                        self.utp.projective.append(self.head_pose)

                        self.utpa.numUsers = len(frame.users)
                        self.utpa.users.append(self.utp)

                        self.pub.publish(self.utpa)
                        print 'publish topics ======================================================'
                        self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('skeleton_server')
    sk = Skeleton()
    try:
        sk.skeleton_tracker()
    except rospy.ROSInterruptException:
        pass
