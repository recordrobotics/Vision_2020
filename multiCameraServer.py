#!/usr/bin/env python3
#----------------------------------------------------------------------------
# Copyright (c) 2018 FIRST. All Rights Reserved.
# Open Source Software - may be modified and shared by FRC teams. The code
# must be accompanied by the FIRST BSD license file in the root directory of
# the project.
#----------------------------------------------------------------------------

import json
import time
import sys
import Distance

from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from networktables import NetworkTables
import cv2 as cv
import ntcore
import numpy as np

### RUN THIS CODE ON THE RASPBERRY PI
### VISION PROCESSING NOT IMPLEMENTED

print("test")
cs = CameraServer.getInstance()
cs.enableLogging()

# Capture from the first USB Camera on the system
camera = cs.startAutomaticCapture()
camera.setResolution(320, 240)

# Get a CvSink. This will capture images from the camera
cvSink = cs.getVideo()

# (optional) Setup a CvSource. This will send images back to the Dashboard
outputStream = cs.putVideo("SmartDashboard", 320, 240)

# Allocating new images is very expensive, always try to preallocate
img = np.zeros(shape=(320, 240, 3), dtype=np.uint8)

# As a client to connect to a robot
NetworkTables.initialize(server='10.67.31.2')
dashboard = NetworkTables.getTable('SmartDashboard')

Distance.calibrateBall(Distance.imgs[0], 36)
Distance.setDegPx(Distance.imgs[0], 36)
while True:
    # Tell the CvSink to grab a frame from the camera and put it
    # in the source image.  If there is an error notify the output.
    t, img = cvSink.grabFrame(img)
    #print(img)
    if t == 0:
        # Send the output the error.
        outputStream.notifyError(cvSink.getError())
        print("error")
        #skip the rest of the current iteration
        #continue
    
    grey = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
    outputStream.putFrame(img)

    #basic edge detection as a test
    dashboard.putNumber("Test-Py", 42)

    try:
        dist, theta = Distance.momentsBall(img)
    except:
        dist = -1
        theta = -1

    dashboard.putNumber("Distace to Ball", dist)
    dashboard.putNumber("Angle to Ball", theta)
    print("Distance", dist)
    print("Angle", theta)
    #time.sleep(5)
