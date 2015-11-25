#if you don't run stream.start() it con't read_frame
from primesense import nite2
from primesense import openni2
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt
nite2.initialize()
op=openni2.Device(None)
color_stream = op.create_color_stream()
color_stream.set_video_mode(openni2.c_api.OniVideoMode(pixelFormat=openni2.c_api.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888, resolutionX=640, resolutionY=480, fps=30))
color_stream.start()
print 'ok'
bgr = np.fromstring(color_stream.read_frame().get_buffer_as_uint8(),dtype=np.uint8).reshape(480,640,3)
rgb = cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
print 'ok'
#color_stream.stop()


