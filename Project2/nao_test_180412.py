# -*- encoding: UTF-8 -*-
from naoqi import ALProxy

import qi
import math
import almath
import motion
import time
import numpy as np

ROBOT_IP = "192.168.1.6"
PORT = 9559
METER_INCH_RATIO = 0.0254
DECIMAL_PLACES_PRECISION = 2
CELL_DIMENSION = 21     # inches
LANDMARK_SIZE = 0.06    # meters
MAP_WIDTH = 89     # inches
MAP_HEIGHT = 89     # inches
currentCamera = "cameraTop"


# Intiializes the grid map with 0.5 values
def setupGridMap():
    map = np.arange(MAP_WIDTH * MAP_HEIGHT)
    map = map.reshape((MAP_WIDTH, MAP_HEIGHT))
    np.zeros_like(map)

# Update map from noisy sensor measurements given the robot pose
def updateGridMap(currentX, currentY, poseVector):


def metersToInches(meters):
    return round(meters / METER_INCH_RATIO, DECIMAL_PLACES_PRECISION)

def inchesToMeters(inches):
    return round(inches * METER_INCH_RATIO, DECIMAL_PLACES_PRECISION)

def moveArm(motionProxy, pTarget, pRobotName, pChainName):
    ''' Function to make NAO bump on his Torso or Head with his arm '''

    # Set the fraction of max speed for the arm movement.
    pMaxSpeedFraction = 0.5

    # Define the final position.
    if (pTarget == "Torso"):
        ShoulderPitchAngle = 50
    elif (pTarget == "Head"):
        ShoulderPitchAngle = -50
    else:
        print "ERROR: target is unknown"
        print "Must be Torso or Head"
        print "---------------------"
        exit(1)

    ShoulderRollAngle  = 6
    ElbowYawAngle      = 0
    ElbowRollAngle     = -150

    if pChainName == "LArm":
        pTargetAngles = [ShoulderPitchAngle, +ShoulderRollAngle,
            +ElbowYawAngle, +ElbowRollAngle]
    elif pChainName == "RArm":
        pTargetAngles = [ShoulderPitchAngle, -ShoulderRollAngle,
            -ElbowYawAngle, -ElbowRollAngle]
    else:
        print "ERROR: chainName is unknown"
        print "Must be LArm or RArm"
        print "---------------------"
        exit(1)

    # Set the target angles according to the robot version.
    if (pRobotName == "naoH25") or\
       (pRobotName == "naoAcademics") or\
       (pRobotName == "naoT14"):
        pTargetAngles += [0.0, 0.0]
    elif (pRobotName == "naoH21"):
        pass
    elif (pRobotName == "naoT2"):
        pTargetAngles = []
    else:
        print "ERROR: Your robot is unknown"
        print "This test is not available for your Robot"
        print "---------------------"
        exit(1)

    # Convert to radians.
    pTargetAngles = [ x * motion.TO_RAD for x in pTargetAngles]

    # Move the arm to the final position.
    motionProxy.angleInterpolationWithSpeed(
        pChainName, pTargetAngles, pMaxSpeedFraction)

def moveHead(motionProxy, pTarget, motionType):
    print "Moving my head to " + str(pTarget)
    maxSpeedFraction  = 0.2 # Using 20% of maximum joint speed
    motionProxy.angleInterpolationWithSpeed(motionType, pTarget * motion.TO_RAD, maxSpeedFraction)
    #time.sleep(2.0)

def detectLandmark(landmarkProxy):
    ''' Function to make NAO detect NAO marks '''

    # Subscribe to LandmarkDetected event from ALLandMarkDetection proxy.
    landmarkProxy.subscribe("landmarkTest")

    # Wait for a mark to be detected.
    markData = memoryProxy.getData("LandmarkDetected")
    while (len(markData) == 0):
        markData = memoryProxy.getData("LandmarkDetected")

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

    x = str(metersToInches(robotToLandmark.r1_c4))
    y = str(metersToInches(robotToLandmark.r2_c4))
    z = str(metersToInches(robotToLandmark.r3_c4))

    print "x " + x + " (in inches)"
    print "y " + y + " (in inches)"
    print "z " + z + " (in inches)"

    speakProxy.say("I have detected a landmark.")
    speakProxy.say("It is located at " + x + " inches away on my ex axis")
    speakProxy.say("And at " + y + " inches away on my why axis")
    speakProxy.say("And at " + z + " inches away on my zee axis")

    # Subscribe to sonars, this will launch sonars (at hardware level) and start data acquisition.
    sonarProxy.subscribe("sonarTest")

    # Get sonar left first echo (distance in meters to the first obstacle).
    print "left sonar data = " + str(round(memoryProxy.getData("Device/SubDeviceList/US/Left/Sensor/Value"), DECIMAL_PLACES_PRECISION))

    # Same thing for right.
    print "right sonar data = " + str(round(memoryProxy.getData("Device/SubDeviceList/US/Right/Sensor/Value"), DECIMAL_PLACES_PRECISION))

    speakProxy.say("My left sonar sensor detected at " + str(round(memoryProxy.getData("Device/SubDeviceList/US/Left/Sensor/Value"), DECIMAL_PLACES_PRECISION)))
    speakProxy.say("And my right sonar sensor detected at " + str(round(memoryProxy.getData("Device/SubDeviceList/US/Right/Sensor/Value"), DECIMAL_PLACES_PRECISION)))

    sonarProxy.unsubscribe("sonarTest")
    landmarkProxy.unsubscribe("landmarkTest")

# position robot in center of home cell and localize in Home cell
def localizeHome():

    # Learning home.
    map = localizationProxy.learnHome()
    # Check that no problem occurred.
    if map == 0:
        print "Learning OK"
    else:
        print "Error during learning " + str(ret)

def localizeInCell():
    # look straight ahead i.e. 0 degrees
    moveHead(motionProxy, 0, "HeadYaw")

    # run landmark detection
    detectLandmark(landmarkProxy)

    # look left i.e. 90 degrees
    moveHead(motionProxy, 90, "HeadYaw")

    # run landmark detection
    detectLandmark(landmarkProxy)

    # store localization_left

    # look right i.e. -90 degrees
    moveHead(motionProxy, -90, "HeadYaw")

    # run landmark detection
    detectLandmark(landmarkProxy)

    # store localization_right

    # look center i.e. 0 degrees
    moveHead(motionProxy, 0, "HeadYaw")
    moveHead(motionProxy, 0, "HeadPitch")

def saveLocation():
    # Save the data for later use.
    map = localizationProxy.save("example")
    # Check that no problem occurred.
    if map == 0:
        print "saving OK"
    else:
        print "error during saving" + str(ret)

def transitionCell(inches):
    ''' http://doc.aldebaran.com/2-5/naoqi/motion/control-walk-api.html#ALMotionProxy::moveTo__floatCR.floatCR.floatCR '''
    ''' https://github.com/uts-magic-lab/pynaoqi_mate/blob/master/naoqi_proxy_python_classes/ALVisualCompass.py '''

    # Start a move
    x = inchesToMeters(inches)
    y = 0.0
    theta = 0.0
    #motionProxy.moveTo(x, y, theta, _async=False)
    #motionProxy.waitUntilMoveIsFinished()
    compassProxy.moveTo(x, y, theta)

def rotateInCell(theta):
    ''' http://doc.aldebaran.com/2-5/naoqi/motion/control-walk-api.html#ALMotionProxy::moveTo__floatCR.floatCR.floatCR '''
    ''' https://github.com/uts-magic-lab/pynaoqi_mate/blob/master/naoqi_proxy_python_classes/ALVisualCompass.py '''

    # Start a move
    x = 0.0
    y = 0.0
    theta = theta
    #motionProxy.moveTo(x, y, theta, _async=False)
    #motionProxy.waitUntilMoveIsFinished()
    compassProxy.moveTo(x, y, theta)

def returnHome():
    # Go back home.
    map = localizationProxy.goToHome()
    # Check that no problem occurred.
    if map == 0:
        print "go to home OK"
    else:
        print "error during go to home " + str(ret)

def rest():
    # Go to rest position
    motionProxy.rest()

# Init proxies.
awarenessProxy      = ALProxy("ALBasicAwareness", ROBOT_IP, PORT)
motionProxy         = ALProxy("ALMotion", ROBOT_IP, PORT)
postureProxy        = ALProxy("ALRobotPosture", ROBOT_IP, PORT)
faceProxy           = ALProxy("ALFaceDetection", ROBOT_IP, PORT)
speakProxy          = ALProxy("ALTextToSpeech", ROBOT_IP, PORT)
memoryProxy         = ALProxy("ALMemory", ROBOT_IP, PORT)
landmarkProxy       = ALProxy("ALLandMarkDetection", ROBOT_IP, PORT)
sonarProxy          = ALProxy("ALSonar", ROBOT_IP, PORT)
localizationProxy   = ALProxy("ALLocalization", ROBOT_IP, PORT)
compassProxy        = ALProxy("ALVisualCompass", ROBOT_IP, PORT)

# Wake up robot
motionProxy.wakeUp()

awarenessProxy.setStimulusDetectionEnabled("People", False)
awarenessProxy.setStimulusDetectionEnabled("Sound", False)

# Send NAO to Pose Init
postureProxy.goToPosture("StandInit", 0.5)

# Disable face tracking.
faceProxy.enableTracking(False)

# Prepare to speak
speakProxy.say("Ready to go again!")

setupGridMap()
localizeInCell()
transitionCell(21)
rotateInCell(math.pi / 2)
rest()

speakProxy.say("I am done now. Bye.")

# Send NAO to Pose Init
postureProxy.goToPosture("Crouch", 0.5)
