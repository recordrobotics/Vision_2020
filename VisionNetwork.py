import Distance
import cv2 as cv
import numpy as np
from networktables import NetworkTables
from cscore import CameraServer

def main():
    ### COPIED FROM EXAMPLE 2020-01-25###
    
    cs = CameraServer.getInstance()
    cs.enableLogging()

    # Capture from the first USB Camera on the system
    camera = cs.startAutomaticCapture()
    camera.setResolution(320, 240)

    # Get a CvSink. This will capture images from the camera
    cvSink = cs.getVideo()

    # (optional) Setup a CvSource. This will send images back to the Dashboard
    outputStream = cs.putVideo("Name", 320, 240)

    # Allocating new images is very expensive, always try to preallocate
    img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)

    # As a client to connect to a robot
    NetworkTables.initialize(server='roborio-6731-frc.local')
    dashboard = NetworkTables.getTable('SmartDashboard')

    while True:
        # Tell the CvSink to grab a frame from the camera and put it
        # in the source image.  If there is an error notify the output.
        time, img = cvSink.grabFrame(img)
        if time == 0:
            # Send the output the error.
            outputStream.notifyError(cvSink.getError())
            # skip the rest of the current iteration
            continue
        
        #basic edge detection as a test
        img = cv.Canny(img, 100, 200)
        dashboard.putNumber("Test-Py", 42)

        # (optional) send some image back to the dashboard
        outputStream.putFrame(img)