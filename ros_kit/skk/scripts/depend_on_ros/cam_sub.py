import cv2
import rospy
import os
import sys
import roslib
import time
import rospkg
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from geometry_msgs.mgs import Vector3
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class Face_Service():
        def __init__(self):
                self.bridge = CvBridge()
                self.head_sub = rospy.Subscriber("/skeleton",UserTrackerPoseArray, self.utpa_callback, queue_size = 10)
                self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback, queue_size = 1)
                self.head_pose[] = Vector3()
                self.pixel_center[] = Vector3()
                rospack = rospkg.RosPack()
                haar_data = rospack.get_path('skk') + '/config'
                self.cascPath = os.path.join(haar_data,"haarcascade_frontalface_default.xml")
                self.faceCascade = cv2.CascadeClassifier(self.cascPath)

                self.pub = rospy.Publisher('face_detected', Int8, queue_size=10)
                self.rate = rospy.Rate(10)


        def utpa_callback(self, user_pose_array):
                num = user_pose_array.numUsers
                if num > 0:
                    self.head_pose.append(user_pose_array.users.projective)
                    #format the raw pose data that can match with pixel in image
                    #TODO: find a more accurate transform
		            pixel_x = (head_pose.x * focal_lengh)/self.head_pose.z
		            pixel_y = (-head_pose.y * focal_lengh)/self.head_pose.z
                    
                    meter_z = self.head_pose.z/1000.0

		            self.pixel_center.append(head_pose)
                    self.pixel_center[user_pose_array.numUsers - num].x = optical_x + pixel_x
		            self.pixel_center[user_pose_array.numUsers - num].y = optical_y + pixel_y
                    self.pexel_center[user_pose_array.numUsers - num].z = meter_z
                    print 'pixel_center:', pixel_center

                    num = num - 1

        def image_callback(self, image):
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
                    for pi in self.pixel_center:
                        cv2.rectangle(image, (pi.x - 50, pi.y - 50), (pi.x + 50, pi.y + 50), (255, 0, 0), 2)
                    detection = self.detect_faces(cv_image)
                        cv2.circle(image, (int(pixel_center.x), int(pixel_center.y)), 10, (255,0,0),-1)
                    for (x, y, w, h) in detection:
                        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)

                    cv2.imshow("Face & Head ", image)
                    cv2.waitKey(1)

                except CvBridgeError, e:
                    print e

                    self.pub.publish(len(pi))

        def detect_faces(self, image):
                _scale_Factor=1.2
                _min_Neighbor=5
                _min_Size=(30,30)
                flags = cv2.cv.CV_HAAR_SCALE_IMAGE
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

                faces = self.faceCascade.detectMultiScale(
                        gray,
                        scaleFactor=1.1,
                        minNeighbors=5,
                        minSize=(30, 30),
                        flags=cv2.cv.CV_HAAR_SCALE_IMAGE
                        )

                return faces

if __name__ == '__main__':
        rospy.init_node('face_srv')
        fd = Face_Service()
        rospy.spin()
