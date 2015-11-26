import cv2
import rospy
import os
import sys
import roslib
import time
import rospkg
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from skk.msg import UserTrackerPoseArray
from skk.msg import UserTrackerPose
import message_filters

class Face_Service():
        def __init__(self):
            self.bridge = CvBridge()
            self.head_sub = message_filters.Subscriber("/skeleton", UserTrackerPoseArray)
            self.image_sub = message_filters.Subscriber("/usb_cam/image_raw", Image)
            #ts = message_filters.TimeSynchronizer([self.head_sub, self.image_sub], 10)
            ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.head_sub], 10, 4)
            ts.registerCallback(self.callback)

            self.head_pose = Vector3()
            self.pixel_center = Vector3()
            self.people_number = 0

            #self.pub = rospy.Publisher('face_detected', Int8, queue_size=10)
            self.pub = rospy.Publisher('face_detected', Bool, queue_size=10)
            self.data = Bool()
            self.rate = rospy.Rate(10)


        def callback(self, image, user_pose_array):
            print 'test-c'
            self.people_number = user_pose_array.numUsers
            num = self.people_number
            self.head_pose = Vector3()
            self.pixel_center = Vector3()

            #sub head pose

            while num > 0:
                focal_length = 525.0
                optical_x = 319.5 
                optical_y = 239.5

                #Vector3 class
                self.head_pose = (user_pose_array.users[self.people_number - num].head)
                #print type(user_pose_array.users[self.people_number - num].head)
                
                print 'num:',num
                print 'people_number',self.people_number
                print self.head_pose
                #print len(self.pixel_center)
                self.pixel_center = self.head_pose
                #print len(self.pixel_center)

                #print type(self.pixel_center[self.people_number - num])
                #self.pixel_center[self.people_number - num].x = self.head_pose.x
                #self.pixel_center[self.people_number - num].y = self.head_pose.y
                #self.pexel_center[self.people_number - num].z = self.head_pose.z
                print 'pixel_center:', self.pixel_center
                num = num - 1

            try:
                cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")

                #draw the skeleton center and head rectangle


                num = self.people_number
                while num > 0:
                    #for pi in self.pixel_center[num - 1]:
                    #    cv2.rectangle(cv_image, (int(pi.x - 50), int(pi.y - 50)), (int(pi.x + 50), int(pi.y + 50)), (255, 0, 0), 2)
                    #    cv2.circle(cv_image, (int(pi.x), int(pi.y)), 10, (255,0,0),-1)
                    #num = num - 1

                    pi = self.pixel_center
                    cv2.rectangle(cv_image, (int(pi.x - 50), int(pi.y - 50)), (int(pi.x + 50), int(pi.y + 50)), (255, 0, 0), 2)
                    cv2.circle(cv_image, (int(pi.x), int(pi.y)), 10, (255,0,0),-1)
                    num = num - 1

                #draw the face rectangle
                detection = self.detect_faces(cv_image)
                for (x, y, w, h) in detection:
                    cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

                cv2.imshow("Face & Head ", cv_image)
                cv2.waitKey(1)

            except CvBridgeError, e:
                print e

            #publish how many skeleton detected
            #self.pub.publish(len(self.pixel_center))

        def detect_faces(self, image):
                #parameters
                _scale_Factor=1.2
                _min_Neighbor=5
                _min_Size=(30,30)
                flags = cv2.cv.CV_HAAR_SCALE_IMAGE
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

                #face data_base
                rospack = rospkg.RosPack()
                haar_data = rospack.get_path('skk') + '/config'
                cascPath = os.path.join(haar_data,"haarcascade_frontalface_default.xml")
                faceCascade = cv2.CascadeClassifier(cascPath)

                #detect faces
                faces = faceCascade.detectMultiScale(
                        gray,
                        scaleFactor=1.2,
                        minNeighbors=5,
                        minSize=(30, 30),
                        flags=cv2.cv.CV_HAAR_SCALE_IMAGE
                        )

                return faces

if __name__ == '__main__':
        rospy.init_node('face_srv')
        print 'test'
        fd = Face_Service()
        print 'test2'
        rospy.spin()
