# -*- encoding: UTF-8 -*-
from naoqi import ALProxy

import qi
import math
import almath
import motion
import time
import numpy as np
import matplotlib.pyplot as plt

ROBOT_IP = "192.168.1.6"
PORT = 9559
CELL_DIMENSION = 21     # inches
MAZE_WIDTH_CELLS = 4    # 4 cells wide
MAZE_HEIGHT_CELLS = 4    # 4 cells tall
LANDMARK_SIZE = 0.06    # meters

X_ROBOT = 0;
Y_ROBOT = 0;
HEADING_ROBOT = 0;
NEXT_MOVE = '';

# Init proxies.
awarenessProxy  = ALProxy("ALBasicAwareness", ROBOT_IP, PORT)
motionProxy     = ALProxy("ALMotion", ROBOT_IP, PORT)
postureProxy    = ALProxy("ALRobotPosture", ROBOT_IP, PORT)
faceProxy       = ALProxy("ALFaceDetection", ROBOT_IP, PORT)
speakProxy      = ALProxy("ALTextToSpeech", ROBOT_IP, PORT)
memoryProxy     = ALProxy("ALMemory", ROBOT_IP, PORT)
landmarkProxy   = ALProxy("ALLandMarkDetection", ROBOT_IP, PORT)
sonarProxy      = ALProxy("ALSonar", ROBOT_IP, PORT)

''' Function to setup and initialize NAO without autonomous life '''
def initNao():

    # Wake up robot
    motionProxy.wakeUp()

    # Disable autonomous features
    faceProxy.enableTracking(False)
    awarenessProxy.setStimulusDetectionEnabled("People", False)
    awarenessProxy.setStimulusDetectionEnabled("Movement", False)
    awarenessProxy.setStimulusDetectionEnabled("Sound", False)
    awarenessProxy.stopAwareness();

    # Send NAO to Pose Init
    postureProxy.goToPosture("StandInit", 0.5)

    print postureProxy.getPostureFamily()

    # Set the current camera ("CameraTop" or "CameraBottom").
    cameraTop = "CameraTop"
    cameraBottom = "CameraBottom"

    # Subscribe to LandmarkDetected event from ALLandMarkDetection proxy.
    landmarkProxy.subscribe("landmarkTest")

    speakProxy.say("Yo! Dr. Sealauhghee, Watch This.")

''' Function to setup and initialize map '''
def initMap():
    plt.cla()
    plt.axis([-CELL_DIMENSION * MAZE_WIDTH_CELLS * 1,CELL_DIMENSION * MAZE_WIDTH_CELLS * 1,-CELL_DIMENSION * MAZE_HEIGHT_CELLS * 1,CELL_DIMENSION * MAZE_HEIGHT_CELLS * 1])
    plt.grid(False)
    plt.ion()
    plt.show()

''' Function to convert meters to inches '''
def metersToInches(meters):
    return meters / 0.0254

''' Function to convert inches to meters '''
def inchesToMeters(inches):
    return inches * 0.0254

''' Function to rotate NAO head '''
def rotateHeadDegrees(motionProxy, pTarget, maxSpeedFraction=0.7):
    motionProxy.angleInterpolationWithSpeed("HeadYaw", pTarget * motion.TO_RAD, maxSpeedFraction)
    time.sleep(0.5)

''' Function to make NAO detect NAO marks '''
def detectLandmark(landmarkProxy, currentCamera='CameraTop'):
    x = 0
    y = 0
    z = 0

    # Wait for a mark to be detected.
    cycles = 0
    markData = memoryProxy.getData("LandmarkDetected")
    while (len(markData) == 0):
        markData = memoryProxy.getData("LandmarkDetected")
        #print cycles
        cycles = cycles + 1
        if (cycles > 10):
            return 0, 0, 0
    #print("number of detections = "+str(len(markData)))

    # Retrieve landmark center position in radians.
    wzCamera = markData[1][0][0][1]
    wyCamera = markData[1][0][0][2]

    # Retrieve landmark angular size in radians.
    angularSize = markData[1][0][0][3]

    # Compute distance to landmark.
    distanceFromCameraToLandmark = LANDMARK_SIZE / ( 2 * math.tan( angularSize / 2))

    # Get current camera position in NAO space.
    transform = motionProxy.getTransform(currentCamera, 2, True)
    transformList = almath.vectorFloat(transform)
    robotToCamera = almath.Transform(transformList)

    # Compute the rotation to point towards the landmark.
    cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(0, wyCamera, wzCamera)

    # Compute the translation to reach the landmark.
    cameraToLandmarkTranslationTransform = almath.Transform(distanceFromCameraToLandmark, 0, 0)

    # Combine all transformations to get the landmark position in NAO space.
    robotToLandmark = robotToCamera * cameraToLandmarkRotationTransform * cameraToLandmarkTranslationTransform

    #print "Finish Landmark detect"

    return metersToInches(robotToLandmark.r1_c4), metersToInches(robotToLandmark.r2_c4), metersToInches(robotToLandmark.r3_c4)

''' Function to get median from a sample of NAO marks detections '''
def median_perception(landmarkProxy, n=10):
    x = []
    y = []
    z = []

    for i in range(n):
        temp_x, temp_y, temp_z = detectLandmark(landmarkProxy, currentCamera='CameraTop')
        x.append(temp_x)
        y.append(temp_y)
        z.append(temp_z)
        time.sleep(0.1)

    #return np.median(x), np.median(y), np.median(z)
    return x, y, z

''' Function to localize NAO in cell (and look left and right) '''
def localizeRobot(lr=True):

    # look straight ahead i.e. 0 degrees
    rotateHeadDegrees(motionProxy, 0)

    # run landmark detection
    #forward_x, forward_y, forward_z = detectLandmark(landmarkProxy, 'CameraTop')
    #forward_x, forward_y, forward_z = median_perception(landmarkProxy)
    forward_x, forward_y, forward_z = perceiveMapRobot()

    if lr:
        # look left i.e. 90 degrees
        rotateHeadDegrees(motionProxy, 90)

        # run landmark detection
        #left_x, left_y, left_z = detectLandmark(landmarkProxy, 'CameraTop')
        left_x, left_y, left_z = median_perception(landmarkProxy)

        # store localization_left

        # look right i.e. -90 degrees
        rotateHeadDegrees(motionProxy, -90)

        # run landmark detection
        #right_x, right_y, right_z = detectLandmark(landmarkProxy, 'CameraTop')
        right_x, right_y, right_z = median_perception(landmarkProxy)

        # store localization_right

        # look center i.e. 0 degrees
        rotateHeadDegrees(motionProxy, 0)

    return forward_x, forward_y, forward_z

''' Function to move NAO '''
def moveRobot(x_inches, y_inches):

    # Prep the variables
    x = inchesToMeters(x_inches)
    y = inchesToMeters(y_inches)
    angular_discount_factor = 0.65; # 65% for inaccurate motion odometry (from stochastic derivation)
    rotation_threshold = 10; # min angle of deviation that requires robot rotation

    # Compute robot geometric rotation
    theta = angular_discount_factor * math.atan2(y,x)
    #print("x="+str(x_inches) + ", y="+str(y_inches) + ", theta="+str(math.degrees(theta)))

    # Decide if rotation is necessary
    if (math.fabs(theta) > math.radians(rotation_threshold)) and (x_inches > CELL_DIMENSION/4):

        # rotate robot by theta
        rotateRobot(theta)
        return

    # Move in y-direction - left / right
    motionProxy.moveTo(0, y, 0, _async=False)

    # Move in x-direction - forward / backward
    motionProxy.moveTo(x, 0, 0, _async=False)

    # Wait for robot motion to finish
    motionProxy.waitUntilMoveIsFinished()

''' Function to rotate NAO '''
def rotateRobot(theta, sidestepCorrection=False):
    ''' http://doc.aldebaran.com/2-5/naoqi/motion/control-walk-api.html#ALMotionProxy::moveTo__floatCR.floatCR.floatCR '''

    # Start a rotation
    motionProxy.moveTo(0, 0, theta, _async=False)

    # Move in x-direction - forward / backward
    if (sidestepCorrection):
        motionProxy.moveTo(0, inchesToMeters(-1), 0, _async=False)

    # Wait for it to finish
    motionProxy.waitUntilMoveIsFinished()

''' Function to rest NAO '''
def rest():
    # Go to rest position
    #motionProxy.rest()
    postureProxy.goToPosture("StandInit", 0.3)
    speakProxy.say("I am done now.")
    landmarkProxy.unsubscribe("landmarkTest")

''' Simulated plot of maze '''
def sim_plotter():
    plt.cla()
    plt.axis("equal")
    plt.grid(False)
    plt.xlim(0, CELL_DIMENSION * MAZE_WIDTH_CELLS * 4)
    plt.ylim(0, CELL_DIMENSION * MAZE_HEIGHT_CELLS * 4)

    #initial location in cell (1, 1)
    x, y = CELL_DIMENSION * MAZE_WIDTH_CELLS * 0.03 * np.random.random(size=(2,10)) + CELL_DIMENSION * MAZE_WIDTH_CELLS

    for i in range(0, len(x), 2):
        mapRobot(x[i:i+2], y[i:i+2],0,label='y+')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='south')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='west')

    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='south')
    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='west')
    mapRobot(np.median(x), np.median(y), theta=0, label='gx')

    #move up to cell (1, 2)
    x, y = x, y + CELL_DIMENSION

    for i in range(0, len(x), 2):
        mapRobot(x[i:i+2], y[i:i+2],0,label='y+')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='north')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='west')

    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='north')
    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='west')
    mapRobot(np.median(x), np.median(y), theta=0, label='gx')

    #move right to cell (2, 2)
    x, y = x + CELL_DIMENSION, y

    for i in range(0, len(x), 2):
        mapRobot(x[i:i+2], y[i:i+2],0,label='y+')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='north')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='east')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='south')

    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='north')
    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='east')
    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='south')
    mapRobot(np.median(x), np.median(y), theta=0, label='gx')

    #move left to cell (1, 2)
    x, y = x - CELL_DIMENSION, y

    #move down to cell (1, 1)
    x, y = x, y - CELL_DIMENSION

    #move right to cell (2, 1)
    x, y = x + CELL_DIMENSION, y

    for i in range(0, len(x), 2):
        mapRobot(x[i:i+2], y[i:i+2],0,label='y+')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='north')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='south')

    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='north')
    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='south')
    mapRobot(np.median(x), np.median(y), theta=0, label='gx')

    #move right to cell (3, 1)
    x, y = x + CELL_DIMENSION, y

    for i in range(0, len(x), 2):
        mapRobot(x[i:i+2], y[i:i+2],0,label='y+')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='south')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='east')

    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='south')
    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='east')
    mapRobot(np.median(x), np.median(y), theta=0, label='gx')

    #move up to cell (3, 2)
    x, y = x, y + CELL_DIMENSION

    for i in range(0, len(x), 2):
        mapRobot(x[i:i+2], y[i:i+2],0,label='y+')
        #mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='west')

    #mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='west')
    mapRobot(np.median(x), np.median(y), theta=0, label='gx')

    #move right to cell (4, 2)
    x, y = x + CELL_DIMENSION, y

    for i in range(0, len(x), 2):
        mapRobot(x[i:i+2], y[i:i+2],0,label='y+')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='north')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='east')

    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='north')
    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='east')
    mapRobot(np.median(x), np.median(y), theta=0, label='gx')

    #move down to cell (4, 1)
    x, y = x, y - CELL_DIMENSION

    for i in range(0, len(x), 2):
        mapRobot(x[i:i+2], y[i:i+2],0,label='y+')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='south')
        #mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='west')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='east')

    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='south')
    #mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='west')
    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='east')
    mapRobot(np.median(x), np.median(y), theta=0, label='gx')

    #move up to cell (4, 2)
    x, y = x, y + CELL_DIMENSION

    #move left to cell (3, 2)
    x, y = x - CELL_DIMENSION, y

    #move up to cell (3, 3)
    x, y = x, y + CELL_DIMENSION

    #move up to cell (3, 4)
    x, y = x, y + CELL_DIMENSION

    for i in range(0, len(x), 2):
        mapRobot(x[i:i+2], y[i:i+2],0,label='y+')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='north')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='east')

    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='north')
    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='east')
    mapRobot(np.median(x), np.median(y), theta=0, label='gx')

    #move down to cell (3, 3)
    x, y = x, y - CELL_DIMENSION

    #move left to cell (2, 3)
    x, y = x - CELL_DIMENSION, y

    for i in range(0, len(x), 2):
        mapRobot(x[i:i+2], y[i:i+2],0,label='y+')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='south')

    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='south')
    mapRobot(np.median(x), np.median(y), theta=0, label='gx')

    #move left to cell (1, 3)
    x, y = x - CELL_DIMENSION, y

    for i in range(0, len(x), 2):
        mapRobot(x[i:i+2], y[i:i+2],0,label='y+')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='north')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='west')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='south')

    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='north')
    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='west')
    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='south')
    mapRobot(np.median(x), np.median(y), theta=0, label='gx')

    #move right to cell (2, 3)
    x, y = x + CELL_DIMENSION, y

    #move up to cell (2, 4)
    x, y = x, y + CELL_DIMENSION

    for i in range(0, len(x), 2):
        mapRobot(x[i:i+2], y[i:i+2],0,label='y+')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='north')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='east')

    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='north')
    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='east')
    mapRobot(np.median(x), np.median(y), theta=0, label='gx')

    #move left to cell (1, 4)
    x, y = x - CELL_DIMENSION, y

    for i in range(0, len(x), 2):
        mapRobot(x[i:i+2], y[i:i+2],0,label='y+')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='north')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='west')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='south')

    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='north')
    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='west')
    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='south')
    mapRobot(np.median(x), np.median(y), theta=0, label='gx')

    #move right to cell (2, 4)
    x, y = x + CELL_DIMENSION, y

    #move down to cell (2, 3)
    x, y = x, y - CELL_DIMENSION

    #move right to cell (3, 3)
    x, y = x + CELL_DIMENSION, y

    for i in range(0, len(x), 2):
        mapRobot(x[i:i+2], y[i:i+2],0,label='y+')
    mapRobot(np.median(x), np.median(y), theta=0, label='gx')

    #move right to cell (4, 3)
    x, y = x + CELL_DIMENSION, y

    for i in range(0, len(x), 2):
        mapRobot(x[i:i+2], y[i:i+2],0,label='y+')

    mapRobot(np.median(x), np.median(y), theta=0, label='gx')

    #move up to cell (4, 4)
    x, y = x, y + CELL_DIMENSION

    for i in range(0, len(x), 2):
        mapRobot(x[i:i+2], y[i:i+2],0,label='y+')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='north')
        mapWallFromRobot(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='east')

    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='north')
    mapWallFromRobot(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='east')
    mapRobot(np.median(x), np.median(y), theta=0, label='gx')

    #move down to cell (4, 3)
    x, y = x, y - CELL_DIMENSION

    plt.show()

''' Function to perceive landmarks and plot NAO possible locations '''
def perceiveMapRobot():

    #initial location in cell (1, 1)
    #x, y, z = CELL_DIMENSION * MAZE_WIDTH_CELLS * 0.03 * median_perception(landmarkProxy) + CELL_DIMENSION * MAZE_WIDTH_CELLS
    x, y, z = median_perception(landmarkProxy)

    for i in range(0, len(x), 2):
        mapRobot(x[i], y[i],0,label='y+')

    mapRobot(np.median(x), np.median(y), theta=0, label='gx')

    return np.median(x), np.median(y), np.median(z)

''' Function to plot NAO '''
def mapRobot(x, y, theta=0, label='b+'):
    # Get the robot location
    #print "ROBOT x="+str(x)+", y="+str(y)
    plt.plot(x + X_ROBOT, y-CELL_DIMENSION/2 + Y_ROBOT, label)
    plt.draw()
    plt.pause(0.01)
    plt.show()

''' Function to plot wall '''
def mapWallFromRobot(x, y, label='go-', linewidth=1, landmarkLocation=HEADING_ROBOT):

    print "ROBOT LOCATION="+str(x)+", "+str(y)
    print "WALL HEADING="+str(landmarkLocation)

    x = x + CELL_DIMENSION/2
    y = y - CELL_DIMENSION/2

    if (landmarkLocation==0) :
        # forward
        start = x-CELL_DIMENSION/2, y+CELL_DIMENSION/2
        end = x-CELL_DIMENSION/2, y-CELL_DIMENSION/2
    elif (landmarkLocation==1) :
        # right
        start = x-CELL_DIMENSION/2, y+CELL_DIMENSION/2
        end = x+CELL_DIMENSION/2, y+CELL_DIMENSION/2
    elif (landmarkLocation==2) :
        # back
        start = x+CELL_DIMENSION/2, y+CELL_DIMENSION/2
        end = x+CELL_DIMENSION/2, y-CELL_DIMENSION/2
    elif (landmarkLocation==3) :
        # left
        start = x-CELL_DIMENSION/2, y-CELL_DIMENSION/2
        end = x+CELL_DIMENSION/2, y-CELL_DIMENSION/2

    plt.plot([start[0],end[0]], [start[1],end[1]], label, linewidth)
    plt.draw()
    plt.pause(0.01)
    plt.show()

#==========================================
initNao()
initMap()

for i in range(0, 7):

    # Localize
    x, y, z = localizeRobot()

    distance = (x-CELL_DIMENSION/2)*0.5
    while (distance > 1):
        distance = (x-CELL_DIMENSION/2)*0.5
        moveRobot(distance, y*0.5)
        x, y, z = localizeRobot(lr=False)

        if (HEADING_ROBOT == 0):
            X_ROBOT = X_ROBOT - distance #-
        if (HEADING_ROBOT == 1):
            Y_ROBOT = Y_ROBOT + distance #+
        if (HEADING_ROBOT == 2):
            X_ROBOT = X_ROBOT + distance #+
        if (HEADING_ROBOT == 3):
            Y_ROBOT = Y_ROBOT - distance #-

    # plot forward wall
    mapWallFromRobot(X_ROBOT,Y_ROBOT,label='go-',linewidth=20, landmarkLocation=HEADING_ROBOT)

    print "PLOT FORWARD WALL"
    print "X ROBOT = "+str(X_ROBOT)
    print "Y ROBOT = "+str(Y_ROBOT)
    print "HEADING ROBOT = "+str(HEADING_ROBOT)
    print "=========================================="

    # look left i.e. 90 degrees
    rotateHeadDegrees(motionProxy, 90)
    left_x, left_y, left_z = median_perception(landmarkProxy)

    NEXT_MOVE = 'turn_around'

    if (abs(np.median(left_y)) > 0 and abs(np.median(left_y)) <= CELL_DIMENSION):
        mapWallFromRobot(X_ROBOT, Y_ROBOT,label='bo-',linewidth=20, landmarkLocation=(HEADING_ROBOT - 1) % 4)

        print "PLOT LEFT WALL"
        print "X ROBOT = "+str(X_ROBOT)
        print "Y ROBOT = "+str(Y_ROBOT)
        print "HEADING ROBOT = "+str(HEADING_ROBOT)
        print "=========================================="

    else:
        NEXT_MOVE = 'left'

    # look right i.e. -90 degrees
    rotateHeadDegrees(motionProxy, -90)
    right_x, right_y, right_z = median_perception(landmarkProxy)

    # look forward i.e. 0 degrees
    rotateHeadDegrees(motionProxy, 0)

    if (abs(np.median(right_y)) > 0 and abs(np.median(right_y)) <= CELL_DIMENSION):
        mapWallFromRobot(X_ROBOT, Y_ROBOT,label='ro-',linewidth=20, landmarkLocation=(HEADING_ROBOT + 1) % 4)

        print "PLOT RIGHT WALL"
        print "X ROBOT = "+str(X_ROBOT)
        print "Y ROBOT = "+str(Y_ROBOT)
        print "HEADING ROBOT = "+str(HEADING_ROBOT)
        print "=========================================="

    else:
        NEXT_MOVE = 'right'

    if (NEXT_MOVE == 'left'):
        rotateRobot(math.pi / 2)
        HEADING_ROBOT = (HEADING_ROBOT - 1) % 4

    if (NEXT_MOVE == 'right'):
        rotateRobot(- math.pi / 2)
        HEADING_ROBOT = (HEADING_ROBOT + 1) % 4

    if (NEXT_MOVE == 'turn_around'):
        rotateRobot(- math.pi, sidestepCorrection=True)
        HEADING_ROBOT = (HEADING_ROBOT + 2) % 4

rest()
input("Press [enter] to continue.")
