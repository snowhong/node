import os
import sys 
import ctypes
import cv2 
import time
import numpy as np
from primesense import openni2
from primesense import nite2

#openni2.initialize() 
nite2.initialize()
status = openni2.is_initialized()
if status:
   print 'OpenNI2 Initialization Done !'
else:
   print 'OpenNI2 Initialization Failed !'

ut = nite2.UserTracker(None)
print nite2.is_initialized()
time.sleep(1)
nite2.unload()

dev = openni2.Device(None)



#dev = openni2.Device.open_any()
#print dev._handle
#dev_handle = openni2.c_api.OniDeviceHandle()
#print type(dev)
#print openni2.c_api.OniDeviceHandle
#ut=nite2.UserTracker.open_any()
#ut=nite2.UserTracker(dev)
#print ut
#print type(ut)

