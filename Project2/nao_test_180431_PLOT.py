# -*- encoding: UTF-8 -*-
from naoqi import ALProxy

import qi
import math
import almath
import motion
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as lines

ROBOT_IP = "192.168.1.6"
PORT = 9559
CELL_DIMENSION = 21     # inches
MAZE_WIDTH_CELLS = 4    # 4 cells wide
MAZE_HEIGHT_CELLS = 4    # 4 cells tall
LANDMARK_SIZE = 0.06    # meters

X_GLOBAL = 0;
Y_GLOBAL = 0;
HEADING_GLOBAL = 0;
NEXT_MOVE = '';

def plotRobotMove(x, y, theta=0, label='b+'):
    # Get the robot location
    print "ROBOT x="+str(x)+", y="+str(y)
    plt.plot(x + X_GLOBAL, y-CELL_DIMENSION/2 + Y_GLOBAL, label)
    plt.draw()
    plt.pause(0.01)
    plt.show()

def plotWallFromLandmark(x, y, label='go-', linewidth=1, landmarkLocation=HEADING_GLOBAL):

    print "WALL x="+str(x)+", y="+str(y)
    if (landmarkLocation==0) :
        # west
        start = x-CELL_DIMENSION/2, y
        end = x-CELL_DIMENSION/2, y-CELL_DIMENSION
    elif (landmarkLocation==2) :
        # east
        start = x+CELL_DIMENSION/2, y
        end = x+CELL_DIMENSION/2, y-CELL_DIMENSION
    elif (landmarkLocation==3) :
        # south
        start = x-CELL_DIMENSION/2, y-CELL_DIMENSION
        end = x+CELL_DIMENSION/2, y-CELL_DIMENSION
    else :
        # north
        start = x-CELL_DIMENSION/2, y
        end = x+CELL_DIMENSION/2, y

    plt.plot(
        [
            start[0],
            end[0]
        ], [
            start[1],
            end[1]
        ], label, linewidth
    )
    #plt.plot(150, 150, label)
    plt.draw()
    plt.pause(0.01)
    plt.show()

def metersToInches(meters):
    return meters / 0.0254

def inchesToMeters(inches):
    return inches * 0.0254

def moveHead(motionProxy, pTarget, maxSpeedFraction=0.7):
    ''' Function to make NAO bump on his Torso or Head with his arm '''
    print("Moving my head to " + str(pTarget))

    # Example showing a single target for one joint
    names             = "HeadYaw"
    targetAngles      = pTarget * motion.TO_RAD
    maxSpeedFraction  = maxSpeedFraction # Default: 70% of maximum joint speed
    motionProxy.angleInterpolationWithSpeed(names, targetAngles, maxSpeedFraction)
    time.sleep(2.0)

def detectLandmark(landmarkProxy, currentCamera='CameraTop'):
    ''' Function to make NAO detect NAO marks '''
    print "Starting Landmark detect"
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

    # Retrieve landmark center position in radians.
    #print("number of detections = "+str(len(markData)))
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

    #print("x " + str(round(robotToLandmark.r1_c4/0.0254, 2)) + " (in inches)")
    #print("y " + str(round(robotToLandmark.r2_c4/0.0254, 2)) + " (in inches)")
    #print("z " + str(round(robotToLandmark.r3_c4/0.0254, 2)) + " (in inches)")
    ######################################
    # speakProxy.say("I have detected a landmark.")

    # speakProxy.say("It is located at " + str(round(robotToLandmark.r1_c4/0.0254, 2)) + " inches away on my ex axis")
    # speakProxy.say("And at " + str(round(robotToLandmark.r2_c4/0.0254, 2)) + " inches away on my why axis")
    # speakProxy.say("And at " + str(round(robotToLandmark.r3_c4/0.0254, 2)) + " inches away on my zee axis")

    # Subscribe to sonars, this will launch sonars (at hardware level) and start data acquisition.
    #sonarProxy.subscribe("sonarTest")

    # Get sonar left first echo (distance in meters to the first obstacle).
    #print("left sonar data = " + str(round(memoryProxy.getData("Device/SubDeviceList/US/Left/Sensor/Value"), 2)))

    # Same thing for right.
    #print("right sonar data = " + str(round(memoryProxy.getData("Device/SubDeviceList/US/Right/Sensor/Value"), 2)))

    # speakProxy.say("My left sonar sensor detected at " + str(round(memoryProxy.getData("Device/SubDeviceList/US/Left/Sensor/Value"), 2)))
    # speakProxy.say("And my right sonar sensor detected at " + str(round(memoryProxy.getData("Device/SubDeviceList/US/Right/Sensor/Value"), 2)))

    #sonarProxy.unsubscribe("sonarTest")


    #print "Finish Landmark detect"

    return metersToInches(robotToLandmark.r1_c4), metersToInches(robotToLandmark.r2_c4), metersToInches(robotToLandmark.r3_c4)

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

def localizeCell(lr=True):
    # look straight ahead i.e. 0 degrees
    moveHead(motionProxy, 0)

    # run landmark detection
    #forward_x, forward_y, forward_z = detectLandmark(landmarkProxy, 'CameraTop')
    #forward_x, forward_y, forward_z = median_perception(landmarkProxy)
    forward_x, forward_y, forward_z = plotter()

    if lr:
        # look left i.e. 90 degrees
        moveHead(motionProxy, 90)

        # run landmark detection
        #left_x, left_y, left_z = detectLandmark(landmarkProxy, 'CameraTop')
        left_x, left_y, left_z = median_perception(landmarkProxy)

        # store localization_left

        # look right i.e. -90 degrees
        moveHead(motionProxy, -90)

        # run landmark detection
        #right_x, right_y, right_z = detectLandmark(landmarkProxy, 'CameraTop')
        right_x, right_y, right_z = median_perception(landmarkProxy)

        # store localization_right

        # look center i.e. 0 degrees
        moveHead(motionProxy, 0)

    return forward_x, forward_y, forward_z

def transitionCell(x_inches, y_inches):
    ''' http://doc.aldebaran.com/2-5/naoqi/motion/control-walk-api.html#ALMotionProxy::moveTo__floatCR.floatCR.floatCR '''

    # Load the variables
    x = inchesToMeters(x_inches)
    y = inchesToMeters(y_inches)
    angular_discount_factor = 0.75; # 75% for inaccurate motion odometry (from stochastic derivation)
    rotation_threshold = 10; # min angle of deviation that requires robot rotation

    # Compute robot geometric rotation
    theta = angular_discount_factor * math.atan2(y,x)
    print("x="+str(x_inches) + ", y="+str(y_inches) + ", theta="+str(math.degrees(theta)))

    # Decide if rotation is necessary
    if (math.fabs(theta) > math.radians(rotation_threshold)) and (x_inches > CELL_DIMENSION/4):

        # rotate robot by theta
        rotateInCell(theta)
        return

    # Move in y-direction - left / right
    motionProxy.moveTo(0, y, 0, _async=False)

    # Move in x-direction - forward / backward
    motionProxy.moveTo(x, 0, 0, _async=False)

    # Wait for robot motion to finish
    motionProxy.waitUntilMoveIsFinished()

def rotateInCell(theta):
    ''' http://doc.aldebaran.com/2-5/naoqi/motion/control-walk-api.html#ALMotionProxy::moveTo__floatCR.floatCR.floatCR '''

    # Start a rotation
    motionProxy.moveTo(0, 0, theta, _async=False)

    # Wait for it to finish
    motionProxy.waitUntilMoveIsFinished()

def rest():
    # Go to rest position
    motionProxy.rest()

def sim_plotter():
    plt.cla()
    plt.axis("equal")
    plt.grid(False)
    plt.xlim(0, CELL_DIMENSION * MAZE_WIDTH_CELLS * 4)
    plt.ylim(0, CELL_DIMENSION * MAZE_HEIGHT_CELLS * 4)

    #initial location in cell (1, 1)
    x, y = CELL_DIMENSION * MAZE_WIDTH_CELLS * 0.03 * np.random.random(size=(2,10)) + CELL_DIMENSION * MAZE_WIDTH_CELLS

    for i in range(0, len(x), 2):
        plotRobotMove(x[i:i+2], y[i:i+2],0,label='y+')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='south')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='west')

    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='south')
    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='west')
    plotRobotMove(np.median(x), np.median(y), theta=0, label='gx')

    #move up to cell (1, 2)
    x, y = x, y + CELL_DIMENSION

    for i in range(0, len(x), 2):
        plotRobotMove(x[i:i+2], y[i:i+2],0,label='y+')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='north')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='west')

    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='north')
    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='west')
    plotRobotMove(np.median(x), np.median(y), theta=0, label='gx')

    #move right to cell (2, 2)
    x, y = x + CELL_DIMENSION, y

    for i in range(0, len(x), 2):
        plotRobotMove(x[i:i+2], y[i:i+2],0,label='y+')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='north')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='east')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='south')

    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='north')
    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='east')
    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='south')
    plotRobotMove(np.median(x), np.median(y), theta=0, label='gx')

    #move left to cell (1, 2)
    x, y = x - CELL_DIMENSION, y

    #move down to cell (1, 1)
    x, y = x, y - CELL_DIMENSION

    #move right to cell (2, 1)
    x, y = x + CELL_DIMENSION, y

    for i in range(0, len(x), 2):
        plotRobotMove(x[i:i+2], y[i:i+2],0,label='y+')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='north')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='south')

    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='north')
    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='south')
    plotRobotMove(np.median(x), np.median(y), theta=0, label='gx')

    #move right to cell (3, 1)
    x, y = x + CELL_DIMENSION, y

    for i in range(0, len(x), 2):
        plotRobotMove(x[i:i+2], y[i:i+2],0,label='y+')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='south')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='east')

    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='south')
    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='east')
    plotRobotMove(np.median(x), np.median(y), theta=0, label='gx')

    #move up to cell (3, 2)
    x, y = x, y + CELL_DIMENSION

    for i in range(0, len(x), 2):
        plotRobotMove(x[i:i+2], y[i:i+2],0,label='y+')
        #plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='west')

    #plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='west')
    plotRobotMove(np.median(x), np.median(y), theta=0, label='gx')

    #move right to cell (4, 2)
    x, y = x + CELL_DIMENSION, y

    for i in range(0, len(x), 2):
        plotRobotMove(x[i:i+2], y[i:i+2],0,label='y+')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='north')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='east')

    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='north')
    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='east')
    plotRobotMove(np.median(x), np.median(y), theta=0, label='gx')

    #move down to cell (4, 1)
    x, y = x, y - CELL_DIMENSION

    for i in range(0, len(x), 2):
        plotRobotMove(x[i:i+2], y[i:i+2],0,label='y+')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='south')
        #plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='west')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='east')

    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='south')
    #plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='west')
    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='east')
    plotRobotMove(np.median(x), np.median(y), theta=0, label='gx')

    #move up to cell (4, 2)
    x, y = x, y + CELL_DIMENSION

    #move left to cell (3, 2)
    x, y = x - CELL_DIMENSION, y

    #move up to cell (3, 3)
    x, y = x, y + CELL_DIMENSION

    #move up to cell (3, 4)
    x, y = x, y + CELL_DIMENSION

    for i in range(0, len(x), 2):
        plotRobotMove(x[i:i+2], y[i:i+2],0,label='y+')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='north')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='east')

    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='north')
    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='east')
    plotRobotMove(np.median(x), np.median(y), theta=0, label='gx')

    #move down to cell (3, 3)
    x, y = x, y - CELL_DIMENSION

    #move left to cell (2, 3)
    x, y = x - CELL_DIMENSION, y

    for i in range(0, len(x), 2):
        plotRobotMove(x[i:i+2], y[i:i+2],0,label='y+')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='south')

    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='south')
    plotRobotMove(np.median(x), np.median(y), theta=0, label='gx')

    #move left to cell (1, 3)
    x, y = x - CELL_DIMENSION, y

    for i in range(0, len(x), 2):
        plotRobotMove(x[i:i+2], y[i:i+2],0,label='y+')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='north')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='west')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='south')

    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='north')
    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='west')
    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='south')
    plotRobotMove(np.median(x), np.median(y), theta=0, label='gx')

    #move right to cell (2, 3)
    x, y = x + CELL_DIMENSION, y

    #move up to cell (2, 4)
    x, y = x, y + CELL_DIMENSION

    for i in range(0, len(x), 2):
        plotRobotMove(x[i:i+2], y[i:i+2],0,label='y+')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='north')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='east')

    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='north')
    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='east')
    plotRobotMove(np.median(x), np.median(y), theta=0, label='gx')

    #move left to cell (1, 4)
    x, y = x - CELL_DIMENSION, y

    for i in range(0, len(x), 2):
        plotRobotMove(x[i:i+2], y[i:i+2],0,label='y+')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='north')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='west')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='south')

    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='north')
    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='west')
    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='south')
    plotRobotMove(np.median(x), np.median(y), theta=0, label='gx')

    #move right to cell (2, 4)
    x, y = x + CELL_DIMENSION, y

    #move down to cell (2, 3)
    x, y = x, y - CELL_DIMENSION

    #move right to cell (3, 3)
    x, y = x + CELL_DIMENSION, y

    for i in range(0, len(x), 2):
        plotRobotMove(x[i:i+2], y[i:i+2],0,label='y+')
    plotRobotMove(np.median(x), np.median(y), theta=0, label='gx')

    #move right to cell (4, 3)
    x, y = x + CELL_DIMENSION, y

    for i in range(0, len(x), 2):
        plotRobotMove(x[i:i+2], y[i:i+2],0,label='y+')

    plotRobotMove(np.median(x), np.median(y), theta=0, label='gx')

    #move up to cell (4, 4)
    x, y = x, y + CELL_DIMENSION

    for i in range(0, len(x), 2):
        plotRobotMove(x[i:i+2], y[i:i+2],0,label='y+')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='north')
        plotWallFromLandmark(x[i:i+2], y[i:i+2],label='yo-.',linewidth=1, landmarkLocation='east')

    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='north')
    plotWallFromLandmark(np.median(x), np.median(y),label='go-',linewidth=20, landmarkLocation='east')
    plotRobotMove(np.median(x), np.median(y), theta=0, label='gx')

    #move down to cell (4, 3)
    x, y = x, y - CELL_DIMENSION

    plt.show()

def plotter():

    #initial location in cell (1, 1)
    #x, y, z = CELL_DIMENSION * MAZE_WIDTH_CELLS * 0.03 * median_perception(landmarkProxy) + CELL_DIMENSION * MAZE_WIDTH_CELLS
    x, y, z = median_perception(landmarkProxy)

    for i in range(0, len(x), 2):
        plotRobotMove(x[i], y[i],0,label='y+')

    plotRobotMove(np.median(x), np.median(y), theta=0, label='gx')

    return np.median(x), np.median(y), np.median(z)

#sim_plotter()

# Init proxies.
awarenessProxy  = ALProxy("ALBasicAwareness", ROBOT_IP, PORT)
motionProxy     = ALProxy("ALMotion", ROBOT_IP, PORT)
postureProxy    = ALProxy("ALRobotPosture", ROBOT_IP, PORT)
faceProxy       = ALProxy("ALFaceDetection", ROBOT_IP, PORT)
speakProxy      = ALProxy("ALTextToSpeech", ROBOT_IP, PORT)
memoryProxy     = ALProxy("ALMemory", ROBOT_IP, PORT)
landmarkProxy   = ALProxy("ALLandMarkDetection", ROBOT_IP, PORT)
sonarProxy      = ALProxy("ALSonar", ROBOT_IP, PORT)

# Wake up robot
motionProxy.wakeUp()

# Disable autonomous features
faceProxy.enableTracking(False)
awarenessProxy.setStimulusDetectionEnabled("People", False)
awarenessProxy.setStimulusDetectionEnabled("Movement", False)
awarenessProxy.setStimulusDetectionEnabled("Sound", False)
awarenessProxy.stopAwareness();

# Send NAO to Pose Init
postureProxy.goToPosture("StandInit", 1.0)

print postureProxy.getPostureFamily()

# Prepare to speak
#speakProxy.say("Ready to go again!")

# Set the current camera ("CameraTop" or "CameraBottom").
cameraTop = "CameraTop"
cameraBottom = "CameraBottom"

# Subscribe to LandmarkDetected event from ALLandMarkDetection proxy.
landmarkProxy.subscribe("landmarkTest")

#speakProxy.say("Yo Dr. Silaghi, I'm about localize in next cell.")

# Init plotter
plt.cla()
#plt.axis("equal")
plt.axis([-150,150,-150,150])
plt.grid(False)
#plt.xlim(0, CELL_DIMENSION * MAZE_WIDTH_CELLS * 10)
#plt.ylim(0, CELL_DIMENSION * MAZE_HEIGHT_CELLS * 10)
plt.ion()
plt.show()

for i in range(0, 4):

    # Localize
    x, y, z = localizeCell()

    distance = (x-CELL_DIMENSION/2)*0.5
    while (distance > 1):
        # print x
        # print y
        # print z
        distance = (x-CELL_DIMENSION/2)*0.5
        transitionCell(distance, y*0.5)
        x, y, z = localizeCell(lr=False)

        if (HEADING_GLOBAL == 0):
            X_GLOBAL = X_GLOBAL - distance #-
        if (HEADING_GLOBAL == 1):
            Y_GLOBAL = Y_GLOBAL + distance #+
        if (HEADING_GLOBAL == 2):
            X_GLOBAL = X_GLOBAL + distance #+
        if (HEADING_GLOBAL == 3):
            Y_GLOBAL = Y_GLOBAL - distance #-

    # plot wall
    plotWallFromLandmark(np.median(x)+X_GLOBAL, np.median(y)+Y_GLOBAL,label='go-',linewidth=20, landmarkLocation=HEADING_GLOBAL)

    # look left i.e. 90 degrees
    moveHead(motionProxy, 90)

    # run landmark detection
    left_x, left_y, left_z = median_perception(landmarkProxy)
    print "LEFT X = "+str(left_x)
    print "LEFT Y = "+str(left_y)

    if (abs(np.median(left_y)) > 0 and abs(np.median(left_y)) <= CELL_DIMENSION):
        #plotWallFromLandmark(np.median(left_x)+(CELL_DIMENSION/2)-X_GLOBAL, np.median(left_y)-(CELL_DIMENSION/2)+Y_GLOBAL,label='bo-',linewidth=20,
        #landmarkLocation=(HEADING_GLOBAL + 1) % 4)
        plotWallFromLandmark(X_GLOBAL-(CELL_DIMENSION/2),Y_GLOBAL,label='bo-',linewidth=20,
        landmarkLocation=(HEADING_GLOBAL + 1) % 4)
    else:
        NEXT_MOVE = 'left'

    # look right i.e. -90 degrees
    moveHead(motionProxy, -90)

    # run landmark detection
    right_x, right_y, right_z = median_perception(landmarkProxy)
    print "RIGHT X = "+str(right_x)
    print "RIGHT Y = "+str(right_y)

    # look forward i.e. 0 degrees
    moveHead(motionProxy, 0)

    if (abs(np.median(right_y)) > 0 and abs(np.median(right_y)) <= CELL_DIMENSION):
        #plotWallFromLandmark(np.median(right_x)+(CELL_DIMENSION/2)-X_GLOBAL, np.median(right_y)+(CELL_DIMENSION/2)+Y_GLOBAL,label='ro-',linewidth=20, #landmarkLocation=(HEADING_GLOBAL - 1) % 4)
        plotWallFromLandmark(X_GLOBAL+(CELL_DIMENSION/2)+21,Y_GLOBAL,label='ro-',linewidth=20, landmarkLocation=(HEADING_GLOBAL - 1) % 4)
    else:
        NEXT_MOVE = 'right'

    if (NEXT_MOVE == 'left'):
        rotateInCell(math.pi / 2)
        HEADING_GLOBAL = (HEADING_GLOBAL - 1) % 4

    if (NEXT_MOVE == 'right'):
        rotateInCell(- math.pi / 2)
        HEADING_GLOBAL = (HEADING_GLOBAL + 1) % 4



#rest()

landmarkProxy.unsubscribe("landmarkTest")

speakProxy.say("I am done now. I am localized in the new cell.")

# Send NAO to Pose Init
#postureProxy.goToPosture("StandInit", 1.0)

input("Press [enter] to continue.")
