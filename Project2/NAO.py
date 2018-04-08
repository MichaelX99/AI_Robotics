from naoqi import ALProxy
import math as m
import almath
import motion
import time

import cv2
import matplotlib.pyplot as plt
from scipy import misc
import numpy as np

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

    def retrieve_circle_pixels(self, x, y, r):
        pixels = []

        d = r * 2
        for i in range(d):
            for j in range(d):
                x_p = x - r + i
                y_p = y - r + j
                dx = (x - x_p)**2
                dy = (y - y_p)**2
                if dx + dy <= r**2:
                    pixels.append([x_p, y_p])

        return pixels

    def color_hist(self, img, nbins=32, bins_range=(0, 256)):
        # Compute the histogram of the color channels separately
        channel1_hist = np.histogram(img[:,:,0], bins=nbins, range=bins_range)
        channel2_hist = np.histogram(img[:,:,1], bins=nbins, range=bins_range)
        channel3_hist = np.histogram(img[:,:,2], bins=nbins, range=bins_range)
        # Concatenate the histograms into a single feature vector
        hist_features = np.concatenate((channel1_hist[0], channel2_hist[0], channel3_hist[0]))
        # Return the individual histograms, bin_centers and feature vector
        return hist_features

    def detect(self, color_img, debug=False):
        # convert the image to grayscale
        img = cv2.cvtColor(color_img, cv2.COLOR_RGB2GRAY)

        # reshape the image to make the circles circular and not ellipses
        shape = img.shape
        scale = 8
        new_shape = (int(shape[0]/scale), int(shape[1]/scale*1.75))
        img = cv2.resize(img, new_shape)
        color_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)
        color_img = cv2.resize(color_img, new_shape)

        # blur the image to eliminate false positives
        img = cv2.GaussianBlur(img, (5,5), 0)

        # find the circles in the image
        circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT, 20, .1, minRadius=1, maxRadius=25)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")

            if debug:
                output = img.copy()
                for (x, y, r) in circles:
                    cv2.circle(output, (x, y), r, (0, 255, 0), 4)
                    cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

                # show the output image
                cv2.imshow("output", np.hstack([img, output]))
                cv2.waitKey(0)

                output = np.zeros_like(color_img)

                for (x, y, r) in circles:
                    pixels = self.retrieve_circle_pixels(x, y, r)
                    for p in pixels:
                        x_p = p[0]
                        y_p = p[1]
                        if x_p < img.shape[1] and y_p < img.shape[0]:
                            output[y_p, x_p] = color_img[y_p, x_p]

                cv2.imshow("output", np.hstack([color_img, output]))
                cv2.waitKey(0)

            for (x,y,r) in circles:
                ROI = np.zeros((2*r,2*r,3))
                pixels = self.retrieve_circle_pixels(x, y, r)
                for p in pixels:
                    x_p = p[0]
                    y_p = p[1]
                    if x_p < color_img.shape[1] and y_p < color_img.shape[0]:
                        x_r = x_p - x + r
                        y_r = y_p - y + r
                        print((x_p, x, x_r))
                        #print((color_img[y_p, x_p,0], color_img[y_p, x_p,1], color_img[y_p, x_p,2]))
                        ROI[y_r, x_r,0] = color_img[y_p, x_p,0]
                        ROI[y_r, x_r,1] = color_img[y_p, x_p,1]
                        ROI[y_r, x_r,2] = color_img[y_p, x_p,2]

                #for i in range(len(ROI)):
                #    for j in range(len(ROI[i])):
                #        print ROI[i][j]
                cv2.imshow("roi", ROI)
                #cv2.waitKey(0)
                print("----------------------------------------------\n")

            cv2.imshow("color", color_img)
            #cv2.waitKey(0)

if __name__ == "__main__":
    nao_ip = "192.168.1.4"
    Nao = NAO(nao_ip)

    filepath = "./img.jpg"
    img = misc.imread(filepath)
    Nao.detect(img)#, True)

    #r = 5
    #pixels = Nao.retrieve_circle_pixels(5,5,r)
    #for p in pixels:
    #    print p
    #print len(pixels) / r**2

    #Nao.run()
