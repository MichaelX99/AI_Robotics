from naoqi import ALProxy
import math as m
import almath
import motion
import time

import cv2
import matplotlib.pyplot as plt
from scipy import misc

class NAO(object):
    def __init__(self, ip, connected=False):
        # Define the port # and ip address
        self.PORT = 9559
        self.ROBOT_IP = ip

        # Define the world dimensions relevant to the map
        self.CELL_DIMENSION = 21     # inches
        self.LANDMARK_SIZE = 0.06    # meters

        # Init proxies.
        if connected:
            self.motionProxy = ALProxy("ALMotion", self.ROBOT_IP, self.PORT)
            self.postureProxy = ALProxy("ALRobotPosture", self.ROBOT_IP, self.PORT)
            self.faceProxy = ALProxy("ALFaceDetection", self.ROBOT_IP, self.PORT)
            self.sonarProxy = ALProxy("ALSonar", ROBOT_IP, PORT)


    def stiffnessOn(self):
        # Use the "Body" name to signify the collection of all joints
        pNames = "Body"
        pStiffnessLists = 1.0
        pTimeLists = 1.0
        self.motionProxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

    def metersToInches(self, meters):
        return meters / 0.0254

    def inchesToMeters(self, inches):
        return inches * 0.0254

    def moveArm(self, pTarget, pRobotName, pChainName):
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
        self.motionProxy.angleInterpolationWithSpeed(
            pChainName, pTargetAngles, pMaxSpeedFraction)

    def moveHead(self, pTarget):
        ''' Function to make NAO bump on his Torso or Head with his arm '''
        print "Moving my head to " + str(pTarget)

        # Example showing a single target for one joint
        names             = "HeadYaw"
        targetAngles      = pTarget * motion.TO_RAD
        maxSpeedFraction  = 0.2 # Using 20% of maximum joint speed
        self.motionProxy.angleInterpolationWithSpeed(names, targetAngles, maxSpeedFraction)
        time.sleep(2.0)

    def detectLandmark(self):
        ''' Function to make NAO detect NAO marks '''

        # Subscribe to LandmarkDetected event from ALLandMarkDetection proxy.
        self.landmarkProxy.subscribe("landmarkTest")

        # Wait for a mark to be detected.
        markData = self.memoryProxy.getData("LandmarkDetected")

        while (len(markData) == 0):
            markData = self.memoryProxy.getData("LandmarkDetected")

        # Retrieve landmark center position in radians.
        wzCamera = markData[1][0][0][1]
        wyCamera = markData[1][0][0][2]

        # Retrieve landmark angular size in radians.
        angularSize = markData[1][0][0][3]

        # Compute distance to landmark.
        distanceFromCameraToLandmark = self.LANDMARK_SIZE / ( 2 * m.tan( angularSize / 2))

        # Get current camera position in NAO space.
        transform = self.motionProxy.getTransform(cameraTop, 2, True)
        transformList = almath.vectorFloat(transform)
        robotToCamera = almath.Transform(transformList)

        # Compute the rotation to point towards the landmark.
        cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(0, wyCamera, wzCamera)

        # Compute the translation to reach the landmark.
        cameraToLandmarkTranslationTransform = almath.Transform(distanceFromCameraToLandmark, 0, 0)

        # Combine all transformations to get the landmark position in NAO space.
        robotToLandmark = robotToCamera * cameraToLandmarkRotationTransform * cameraToLandmarkTranslationTransform

        x = round(robotToLandmark.r1_c4/0.0254, 2)
        y = round(robotToLandmark.r2_c4/0.0254, 2)
        z = round(robotToLandmark.r3_c4/0.0254, 2)

        print "x " + str(x) + " (in inches)"
        print "y " + str(y) + " (in inches)"
        print "z " + str(z) + " (in inches)"


        # Subscribe to sonars, this will launch sonars (at hardware level) and start data acquisition.
        self.sonarProxy.subscribe("sonarTest")

        l = round(memoryProxy.getData("Device/SubDeviceList/US/Left/Sensor/Value"), 2)
        r = round(memoryProxy.getData("Device/SubDeviceList/US/Right/Sensor/Value"), 2)

        return x, y, z, l, r

    def transitionCell(self):
        ''' http://doc.aldebaran.com/2-5/naoqi/motion/control-walk-api.html#ALMotionProxy::moveTo__floatCR.floatCR.floatCR '''

        # Start a move
        x = inchesToMeters(21)
        y = 0.0
        theta = 0.0
        self.motionProxy.moveTo(x, y, theta, _async=True)

        # Wait for it to finish
        self.motionProxy.waitUntilMoveIsFinished()

        # Go to rest position
        self.motionProxy.rest()

    def run(self):
        # Wake up robot
        self.motionProxy.wakeUp()

        # Set NAO in Stiffness On
        stiffnessOn()




if __name__ == "__main__":
    nao_ip = "192.168.1.4"
    Nao = NAO(nao_ip)

    #Nao.run()
