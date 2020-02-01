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
camera.setResolution(160, 120)

# Get a CvSink. This will capture images from the camera
cvSink = cs.getVideo()

# (optional) Setup a CvSource. This will send images back to the Dashboard
outputStream = cs.putVideo("SmartDashboard", 160, 120)

# Allocating new images is very expensive, always try to preallocate
img = np.zeros(shape=(160, 120, 3), dtype=np.uint8)

# As a client to connect to a robot
NetworkTables.initialize(server='10.67.31.2')
dashboard = NetworkTables.getTable('SmartDashboard')

while True:
    # Tell the CvSink to grab a frame from the camera and put it
    # in the source image.  If there is an error notify the output.
    time, img = cvSink.grabFrame(img)
    #print(img)
    if time == 0:
        # Send the output the error.
        #outputStream.notifyError(cvSink.getError())
        # skip the rest of the current iteration
        print("error")
        continue
    
    #basic edge detection as a test
    grey = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
    dashboard.putNumber("Test-Py", 42)
    print("iteration")

    # (optional) send some image back to the dashboard
    outputStream.putFrame(grey)