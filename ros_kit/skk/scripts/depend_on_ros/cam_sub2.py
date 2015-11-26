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
                self.image_sub = message_filters.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size = 10) 

                self.head_pose = []
                self.pixel_center = []
                self.people_number = 0

                self.pub = rospy.Publisher('face_detected', Int8, queue_size=10)
                self.rate = rospy.Rate(10)

                rospy.spin()

        def image_callback(self, image):
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")

                    #draw the skeleton center and head rectangle

                    num = self.people_number
                    print 'len: ',len(self.pixel_center)

                    print num
                    while num > 0:
                        for pi in self.pixel_center[num - 1]:
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
                self.pub.publish(self.pixel_center)
                self.pixel_center = []

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
        try:
            print 'test'
            fd = Face_Service()
            print 'test2'
        except rospy.ROSInterruptException: pass
