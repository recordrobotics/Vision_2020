import cv2 as cv
import numpy as np
import math

#load images
imgs = [cv.imread("OtherImgs\Ball_2ft.jpg"), cv.imread("OtherImgs\Ball_3ft.jpg"), cv.imread("OtherImgs\Ball_4ft.jpg"), cv.imread("OtherImgs\Ball_19deg.jpg")]

focalLength = 0
degpx = 0
diagFOV = 68.5
ballWidth = 7

def maskBalls(image):
    image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    #image = cv.GaussianBlur(image, (5,5), 0)
    #define colors
    upperLimit = np.array([60,255,255])
    lowerLimit = np.array([20,100,100])

    mask = cv.inRange(image.copy(), lowerLimit, upperLimit)

    return mask

def findCircle(image):
    #find circles in the masked image, return a center and a radius
    image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    image = cv.medianBlur(image, 19)
    #DO NOT CHANGE VALUES OF CIRCLE TRANSFORM. Param1 = 28, Param2 = 30
    circles = cv.HoughCircles(image, cv.HOUGH_GRADIENT, 1, 100, param1=28, param2=30, minRadius=0, maxRadius=0) 
    return circles

def findRect(image):
    gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    gray = cv.GaussianBlur(gray, (5, 5), 0)
    edge = cv.Canny(gray, 35, 125)

    __, contours, __ = cv.findContours(edge.copy(), cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
    c = max(contours, key = cv.contourArea)

    return cv.minAreaRect(c)

def findDistance(width, focal, perWidth):
    return (width * focal) / perWidth

def calibrate(image, dist):
    global focalLength
    global width
    masked = maskBalls(image)
    masked = cv.bitwise_and(image, image, mask = masked)
    
    circle = findCircle(masked)
    radius = circle[0][0][2]
    #print(radius)

    focalLength = (radius * 2 * dist)/ballWidth

def setDegPx(image):
    global degpx

    width = image.shape[1]
    height = image.shape[0]
    diag = math.sqrt(width**2 + height**2)
    #print("diag", diag)

    degpx = diagFOV/diag

def findAngle(point, image):
    center = [image.shape[0]/2, image.shape[1]/2]
    #print("center ", center)
    #print("point", point)
    return math.sqrt(abs(center[0] - point[0])**2 + abs(center[1] - point[1])**2) * degpx

def distanceToBall(image):
    masked = maskBalls(image)
    masked = cv.bitwise_and(image, image, mask = masked)

    circle = findCircle(masked)
    radius = circle[0][0][2]
    center = [circle[0][0][1], circle[0][0][0]]

    #draw and display circles as well
    cv.circle(image, (circle[0][0][0], circle[0][0][1]), radius, (0, 255, 0), 3)
    #cv.line(image, image.shape[])

    cv.imshow("circle", image)
    cv.waitKey(0)
    cv.destroyAllWindows()

    #calculate angles and distances
    angle = findAngle(center, image)
    xDist = findDistance(ballWidth, focalLength, 2 * radius)
    distance = xDist / (math.cos(math.radians(angle)))

    return distance, angle

setDegPx(imgs[1])
calibrate(imgs[1], 36)
print(distanceToBall(imgs[1]))

