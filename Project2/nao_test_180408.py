# -*- encoding: UTF-8 -*-
from naoqi import ALProxy

import qi
import math
import almath
import motion
import time

ROBOT_IP = "192.168.1.4"
PORT = 9559
CELL_DIMENSION = 21     # inches
LANDMARK_SIZE = 0.06    # meters

def stiffnessOn(proxy):
    # "Body" name = the collection of all joints
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

def metersToInches(meters):
    return meters / 0.0254

def inchesToMeters(inches):
    return inches * 0.0254


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

def moveHead(motionProxy, pTarget):
    ''' Function to make NAO bump on his Torso or Head with his arm '''
    print "Moving my head to " + str(pTarget)

    # Example showing a single target for one joint
    names             = "HeadYaw"
    targetAngles      = pTarget * motion.TO_RAD
    maxSpeedFraction  = 0.2 # Using 20% of maximum joint speed
    motionProxy.angleInterpolationWithSpeed(names, targetAngles, maxSpeedFraction)
    time.sleep(2.0)

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
    transform = motionProxy.getTransform(cameraTop, 2, True)
    transformList = almath.vectorFloat(transform)
    robotToCamera = almath.Transform(transformList)

    # Compute the rotation to point towards the landmark.
    cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(0, wyCamera, wzCamera)

    # Compute the translation to reach the landmark.
    cameraToLandmarkTranslationTransform = almath.Transform(distanceFromCameraToLandmark, 0, 0)

    # Combine all transformations to get the landmark position in NAO space.
    robotToLandmark = robotToCamera * cameraToLandmarkRotationTransform * cameraToLandmarkTranslationTransform

    print "x " + str(round(robotToLandmark.r1_c4/0.0254, 2)) + " (in inches)"
    print "y " + str(round(robotToLandmark.r2_c4/0.0254, 2)) + " (in inches)"
    print "z " + str(round(robotToLandmark.r3_c4/0.0254, 2)) + " (in inches)"

    speakProxy.say("I have detected a landmark.")

    speakProxy.say("It is located at " + str(round(robotToLandmark.r1_c4/0.0254, 2)) + " inches away on my ex axis")
    speakProxy.say("And at " + str(round(robotToLandmark.r2_c4/0.0254, 2)) + " inches away on my why axis")
    speakProxy.say("And at " + str(round(robotToLandmark.r3_c4/0.0254, 2)) + " inches away on my zee axis")

    # Subscribe to sonars, this will launch sonars (at hardware level) and start data acquisition.
    sonarProxy.subscribe("sonarTest")

    # Get sonar left first echo (distance in meters to the first obstacle).
    print "left sonar data = " + str(round(memoryProxy.getData("Device/SubDeviceList/US/Left/Sensor/Value"), 2))

    # Same thing for right.
    print "right sonar data = " + str(round(memoryProxy.getData("Device/SubDeviceList/US/Right/Sensor/Value"), 2))

    speakProxy.say("My left sonar sensor detected at " + str(round(memoryProxy.getData("Device/SubDeviceList/US/Left/Sensor/Value"), 2)))
    speakProxy.say("And my right sonar sensor detected at " + str(round(memoryProxy.getData("Device/SubDeviceList/US/Right/Sensor/Value"), 2)))

    sonarProxy.unsubscribe("sonarTest")
    landmarkProxy.unsubscribe("landmarkTest")

def localizeCell():
    # look straight ahead i.e. 0 degrees
    moveHead(motionProxy, 0)

    # run landmark detection
    detectLandmark(landmarkProxy)

    # look left i.e. 90 degrees
    moveHead(motionProxy, 90)

    # run landmark detection
    detectLandmark(landmarkProxy)

    # store localization_left

    # look right i.e. -90 degrees
    moveHead(motionProxy, -90)

    # run landmark detection
    detectLandmark(landmarkProxy)

    # store localization_right

    # look center i.e. 0 degrees
    moveHead(motionProxy, 0)


def transitionCell(inches):
    ''' http://doc.aldebaran.com/2-5/naoqi/motion/control-walk-api.html#ALMotionProxy::moveTo__floatCR.floatCR.floatCR '''

    # Start a move
    x = inchesToMeters(21)
    y = 0.0
    theta = 0.0
    motionProxy.moveTo(x, y, theta, _async=False)

    # Wait for it to finish
    motionProxy.waitUntilMoveIsFinished()

def rotateInCell(theta):
    ''' http://doc.aldebaran.com/2-5/naoqi/motion/control-walk-api.html#ALMotionProxy::moveTo__floatCR.floatCR.floatCR '''

    # Start a move
    x = 0.0
    y = 0.0
    theta = theta
    motionProxy.moveTo(x, y, theta, _async=False)

    # Wait for it to finish
    motionProxy.waitUntilMoveIsFinished()

def rest():
    # Go to rest position
    motionProxy.rest()

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

awarenessProxy.setStimulusDetectionEnabled("People", False)
awarenessProxy.setStimulusDetectionEnabled("Sound", False)

# Set NAO in Stiffness On
#stiffnessOn(motionProxy)

# Send NAO to Pose Init
postureProxy.goToPosture("StandInit", 0.5)

# Disable face tracking.
faceProxy.enableTracking(False)

# Prepare to speak
speakProxy.say("Ready to go again!")

# Set the current camera ("CameraTop" or "CameraBottom").
cameraTop = "CameraTop"
cameraBottom = "CameraBottom"

localizeCell()
transitionCell(21)
rotateInCell(math.pi / 2)
rest()

speakProxy.say("I am done now. Bye.")

# Send NAO to Pose Init
postureProxy.goToPosture("Crouch", 0.5)
