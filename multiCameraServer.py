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
import numpy as np

print("test")
cs = CameraServer.getInstance()
cs.enableLogging()

# Capture from the first USB Camera on the system
camera = cs.startAutomaticCapture(dev=0)
camera.setResolution(256, 144)

cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_EXPOSURE, -10)

# Get a CvSink. This will capture images from the camera
cvSink = cs.getVideo()

# (optional) Setup a CvSource. This will send images back to the Dashboard
outputStream = cs.putVideo("SmartDashboard", 256, 144)

# Allocating new images is very expensive, always try to preallocate
img = np.zeros(shape=(256, 144, 3), dtype=np.uint8)

# As a client to connect to a robot
NetworkTables.initialize(server='10.67.31.2')
dashboard = NetworkTables.getTable('SmartDashboard')

Distance.calibrateBall(Distance.imgs[0], 36)
Distance.setDegPx(Distance.imgs[0])
while True:
    # Tell the CvSink to grab a frame from the camera and put it
    # in the source image.  If there is an error notify the output.
    t, img = cap.read()
    #print(img)
    '''
    if t == 0:
        # Send the output the error.
        outputStream.notifyError(cvSink.getError())
        print("error")
        #skip the rest of the current iteration
        #continue
    '''
    
    grey = cv.cvtColor(img, cv.COLOR_RGB2GRAY)
    outputStream.putFrame(img)

    #basic edge detection as a test
    dashboard.putNumber("Test-Py", 42)

    try:
        dist, theta = Distance.momentsBall(img)
    except:
        dist = -1
        theta = -1

    try:
        goalDist, goalAngle = Distance.distanceToGoal(img)
    except:
        goalAngle = -1
        goalDist = -1

    dashboard.putNumber("Distace to Ball", dist)
    dashboard.putNumber("Angle to Ball", theta)

    dashboard.putNumber("Distance to Goal", goalDist)
    dashboard.putNumber("Angle to Goal", goalAngle)

    print("Distance", str(dist))
    print("Angle", str(theta))
    time.sleep(0.05)
