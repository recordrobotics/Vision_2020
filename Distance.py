import cv2 as cv
import numpy as np
import math

#load images
imgs = [
    cv.imread("OtherImgs\Ball_2ft.jpg"), 
    cv.imread("OtherImgs\Ball_3ft.jpg"), 
    cv.imread("OtherImgs\Ball_4ft.jpg"), 
    cv.imread("OtherImgs\Ball_19deg.jpg"),
    cv.imread("2020SampleVisionImages\WPILib Robot Vision Images\BlueGoal-084in-Center.jpg"),
    cv.imread("2020SampleVisionImages\WPILib Robot Vision Images\BlueGoal-156in-Center.jpg"),
    cv.imread("2020SampleVisionImages\WPILib Robot Vision Images\BlueGoal-132in-Center.jpg"),
    cv.imread("2020SampleVisionImages\WPILib Robot Vision Images\BlueGoal-180in-Center.jpg"),
    cv.imread("2020SampleVisionImages\WPILib Robot Vision Images\BlueGoal-224in-Center.jpg")
    ]

focalLength = 0
diagpx = 0
hpx = 0
diagFOV = 68.5
horizontalFOV = math.degrees(math.atan(math.tan(math.radians(diagFOV/2)) * (imgs[0].shape[1]/math.sqrt(imgs[0].shape[0]** 2 + imgs[0].shape[1]**2)) * 2))
print(horizontalFOV)
ballWidth = 7
goalWidth = 39.25

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

def findDistance(width, focal, perWidth):
    return (width * focal) / perWidth

def calibrateBall(image, dist):
    global focalLength
    masked = maskBalls(image)
    masked = cv.bitwise_and(image, image, mask = masked)
    
    circle = findCircle(masked)
    radius = circle[0][0][2]
    #print(radius)

    focalLength = (radius * 2 * dist)/ballWidth

def setDegPx(image):
    global diagpx
    global hpx

    width = image.shape[1]
    height = image.shape[0]
    diag = math.sqrt(width**2 + height**2)
    #print("diag", diag)

    diagpx = diagFOV/diag
    hpx = horizontalFOV/width

def findAngle(point, image):
    center = [image.shape[0]/2, image.shape[1]/2]
    return abs(center[1] - point[1])*hpx

def distanceToBall(image):
    masked = maskBalls(image)
    masked = cv.bitwise_and(image, image, mask = masked)

    circle = findCircle(masked)
    
    try:
        radius = circle[0][0][2]
    except:
        return None
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

def calibrateGoal(image, distance):
    global focalLength
    masked = maskGoal(image)
    rect = findGoal(masked)

    focalLength = (rect[2] * distance)/goalWidth

def maskGoal(image):
    image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    upperLimit = np.array([100,255,255])
    lowerLimit = np.array([65,100,100])

    mask = cv.inRange(image.copy(), lowerLimit, upperLimit)

    return mask

def findGoal(mask):
    contours = cv.findContours(cv.Canny(mask.copy(), 30, 200), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)[1]
    contours = max(contours, key = cv.contourArea)
    rect = cv.boundingRect(contours)

    return rect

def distanceToGoal(image):
    mask = maskGoal(image)
    rect = findGoal(mask)

    distance = findDistance(goalWidth, focalLength, rect[2])
    angle = findAngle((rect[0] + rect[2]/2, rect[1]), mask)
    distance = distance / (math.cos(math.radians(angle)))

    return distance, angle

setDegPx(imgs[1])
calibrateBall(imgs[1], 36)
#calibrateGoal(imgs[5], 156)

print("ball", distanceToBall(imgs[4]))
#print("goal", distanceToGoal(imgs[-1]))

