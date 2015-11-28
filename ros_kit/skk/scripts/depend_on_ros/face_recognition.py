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
from pyfaces import pyfaces
import sys,time

if __name__ == "__main__":
    try:
        start = time.time()

        egfaces=int(sys.argv[3])
        thrshld=float(sys.argv[4])
        pyf=pyfaces.PyFaces(imgname,dirname,egfaces,thrshld)

        end = time.time()
        print 'took :',(end-start),'secs'

    except Exception,detail:
        print detail.args

