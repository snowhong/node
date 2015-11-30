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
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
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
        self.sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.sample_cam_callback, queue_size=1)
        #self.pub = rospy.Publisher('skeleton', UserTrackerPoseArray, queue_size=1)
        #self.pub2 = rospy.Publisher('detected', Bool , queue_size=1)
        self.pub3 = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub4 = rospy.Publisher('/face/image_raw', Image, queue_size=1)
        self.rate = rospy.Rate(100)

        self.utpa = UserTrackerPoseArray()
        self.utp = UserTrackerPose()
        self.h = std_msgs.msg.Header()
        self.head_pose = Vector3()
        self.bridge = CvBridge()

        self.detected = False

        rate = rospy.Rate(100)

    def sample_cam_callback(self, image):
        print 'starting capture skeleton'
        is_done = self.skeleton_tracker()
        if is_done:
            print 'sending the head image'
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            pi = self.head_pose
            x1 = int(pi.x - 50)
            y1 = int(pi.y - 50)
            x2 = int(pi.x + 50)
            y2 = int(pi.y + 50)
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.circle(cv_image, (int(pi.x), int(pi.y)), 10, (255,0,0),-1)
            #imh[y:y+h, x:x+w]
            crop_image = cv_image[y1:y1 + (y2 - y1), x1:x1 + (x2 - x1)]
            cv2.imshow("Face & Head ", cv_image)
            cv2.imshow("Crop Head", crop_image)
            cv2.waitKey(1)

            resize_image =  
            image_message = self.bridge.cv2_to_imgmsg(resize_image, encoding="bgr8")
            self.pub4.publish(image_message)

        

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
            dont_pub_blank_msg = 0#Now just let it publish blank message
            self.detected = False

            #Blank Message
            self.h.stamp = rospy.Time.now()
            self.utpa.header = self.h

            for fu in frame.users:
                user = fu
                if user.is_new():
                    self.ut.start_skeleton_tracking(fu.id)
                    print '=============================================================get new user:', fu.id
                #state 2(TRACKED STATES) means skeleton is active, if its a new user don't need to get state.
                elif user.skeleton.state == 0:
                    rotate = Twist()
                    rotate.angular.z = 0.1
                    print 'rotate with vel:(get user but no skeleton now)',rotate
                    self.pub3.publish(rotate)

                elif user.skeleton.state == 2:
                    #head flag
                    head = user.skeleton.joints[0]
                    if(head.positionConfidence > 0.5):
                        print 'user id:', fu.id, 'head pose:', head.position
                        self.detected = True

                        print '==========================',self.detected
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
                #self.pub.publish(self.utpa)
                #self.pub2.publish(self.detected)
                return True
                #print 'self.utpa:', self.utpa

            
	    print '============user in frame',len(frame.users)
            if not len(frame.users):
                rotate = Twist()
                rotate.angular.z = 0.2
                print 'rotate with vel(when no detect people):',rotate
                #self.pub3.publish(rotate)

            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('skeleton_server')
    try:
        sk = Skeleton()
    except rospy.ROSInterruptException:
        pass
    rospy.spin()
