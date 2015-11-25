import cv2
import numpy
from primesense import openni2
from primesense import _openni2 as c_api
from primesense import nite2

openni2.initialize() 
nite2.initialize()
ut=nite2.UserTracker.open_any()

dev = openni2.Device.open_any()
#dev._close()
#dev = openni2.Device.open_any()
#dev._close()
openni2.unload()
