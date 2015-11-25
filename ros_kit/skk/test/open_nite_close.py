from primesense import nite2
from primesense import openni2
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt


nite2.initialize()

ut = nite2.UserTracker(None)

#frame._close()
#print nite2.is_initialized()
#nite2.unload2()

GET_POSE=0
first_time=0
while(1):
    if first_time == 1:
        nite2.initialize()
        ut = nite2.UserTracker(None)
    while(1):
        first_time=1
        time.sleep(0.01)
        frame = ut.read_frame()
        depth_frame = frame.get_depth_frame()
        for fu in frame.users:
            print 'get user!!!!!!!!!!!!!!!!!!!!!!!'
            users = fu
            if users.is_new():
                print 'Get a new user! Start tracking...'
                ut.start_skeleton_tracking(fu.id)
            elif users.skeleton.state == 2:
                head = users.skeleton.joints[0]
                if(head.positionConfidence > 0.5):
                    head_pos = head.position
                    print 'return pose', head_pos
                    nite2.unload2()
                    print 'bre=---------------------------------------akk'
                    GET_POSE=1
                    break
        if GET_POSE == 1:
            break
    print 'as=====================df',head_pos
    
    
    op=openni2.Device(None)
    color_stream = op.create_color_stream()
    color_stream.set_video_mode(openni2.c_api.OniVideoMode(pixelFormat=openni2.c_api.OniPixelFormat.ONI_PIXEL_FORMAT_RGB888, resolutionX=640, resolutionY=480, fps=30))
    color_stream.start()
    
    bgr = np.fromstring(color_stream.read_frame().get_buffer_as_uint8(),dtype=np.uint8).reshape(480,640,3)
    
    rgb = cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
    
    color_stream.stop()
    
    #op.close()#close() doesn't clear handle so we can't create a nitetracker again untill you unload() it 
    openni2.unload()


    _scale_Factor=1.2
    _min_Neighbor=5
    _min_Size=(30,30)
    flags = cv2.cv.CV_HAAR_SCALE_IMAGE
    focal_lengh =  525.0
    optical_x = 319.5
    optical_y = 239.5
    pixel_x = (head_pos.x * focal_lengh)/head_pos.z
    pixel_y = (-head_pos.y * focal_lengh)/head_pos.z
    pixel_center= head_pos
    pixel_center.x = optical_x + pixel_x
    pixel_center.y = optical_y + pixel_y
    print 'pixel_center:',pixel_center
    #100*100 pixel
    head_image = rgb[(pixel_center.x - 50, pixel_center.y - 50), (pixel_center.x + 50, pixel_center.y + 50)] 
    cv2.rectangle(rgb, (int(pixel_center.x - 50), int(pixel_center.y - 50)), (int(pixel_center.x + 50), int(pixel_center.y + 50)), (0, 0, 255), 2)
    
    cv2.circle(rgb, (int(pixel_center.x), int(pixel_center.y)), 10, (255,0,0),-1)
    #gray = cv2.cvtColor(head_image, cv2.COLOR_BGR2GRAY)
    gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
    
    faceCascade = cv2.CascadeClassifier('/home/robot/Robot/src/backup/ros_kit/skk/config/haarcascade_frontalface_default.xml')
    faces = faceCascade.detectMultiScale(
                    gray,
                    _scale_Factor,
                    _min_Neighbor,
                    flags,
                    _min_Size
                    )
    
    for (x, y, w, h) in faces:
        cv2.rectangle(rgb, (x, y), (x+w, y+h), (0, 255, 0), 2)
    
    plt.imshow(rgb)
    plt.show()


#ut=nite2.UserTracker.open_any()#return a usertracker handle and open device
#ut2=nite2.HandTracker.open_any()
#print ut
#print ut2
#print nite2.is_initialized()
#print openni2.is_initialized()

#print openni2._registered_devices

#openni2.initialize()
#op=openni2.Device.open_any() #device handle, device already open so cause error
#stream = op.create_color_stream()
#stream.start()
#stream.stop()
#global test

#for handle in openni2._registered_devices:
#    test = handle
#    print handle
#opex=openni2.Device(None, 1)
#op2=openni2.Device.open_any()

#openni2.Device.enumerate_uris()
#op=openni2.Device(None)
#print op._handle
#openni2.c_api.oniDeviceClose(op._handle)
#openni2.c_api.oniDeviceClose()
#op._close() # will cause segmentation fault (core dump) !!! when excute oniDeviceClose(op._handle)
#op.close()
#print '======closed op'

#print op.enumerate_uris()
#print op.enumerate_uris()

#ut=nite2.UserTracker.open_any()
#ut=nite2.UserTracker(op)
#ut=nite2.UserTracker(None)
#ut.close()

#frame = ut.read_frame()
#depth_frame = frame.get_depth_frame()
#print ut._handle
#nite2.c_api.niteShutdownUserTracker(ut._handle)# will cause segmentation fault (core dump) !!! when excute oniDeviceClose(op._handle)


#for handle in openni2._registered_devices:
#    print handle
#    handle.close()

#exit()
#ut=nite2.UserTracker(op)
#st=openni2.VideoStream()

#ut.close()
#print '=======',nite2.is_initialized()
