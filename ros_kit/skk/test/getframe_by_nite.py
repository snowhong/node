from primesense import nite2
from primesense import openni2
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
nite2.initialize()
ut = nite2.UserTracker(None)

frame = ut.read_frame() #return pnf & handle #pnf:is a sturcture of NiteUserTrackerFrame

depth_frame = frame.get_depth_frame()


#pColorFrame = ctypes.POINTER(openni2.c_api.OniFrame)()
#kcolor_frame = openni2.VideoFrame(pColorFrame)
depth_img = np.fromstring(depth_frame.get_buffer_as_uint8(),dtype=np.uint8).reshape(480,640,3)

plt.imshow(depth_img)
plt.show()

