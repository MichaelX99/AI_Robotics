import urllib2
#import wget
import cv2
import os
import shutil
import numpy as np
import sys
from contextlib import contextmanager
import math as m


michaels_code = "PKPMWTlhvllf"

class R12_Controller:
    def __init__(self, passcode=None):
        # Make sure a passcore was provided
        assert(passcode != None), "Please enter passcode into constructor to use the R12 arm"

        # Create the directories to store the images
        self.cwd = os.getcwd()
        if not os.path.exists(self.cwd + "/Images/"): os.mkdir(self.cwd + "/Images/")
        self.top_dir = self.cwd + "/Images/Top/"
        self.right_dir = self.cwd + "/Images/Right/"
        self.left_dir = self.cwd + "/Images/Left/"
        if os.path.exists(self.top_dir): shutil.rmtree(self.top_dir)
        if os.path.exists(self.right_dir): shutil.rmtree(self.right_dir)
        if os.path.exists(self.left_dir): shutil.rmtree(self.left_dir)
        os.mkdir(self.top_dir)
        os.mkdir(self.right_dir)
        os.mkdir(self.left_dir)

        # Space for commands
        self.space =  "%20"

        # Command starting point
        self.output = "HOME" + self.space

        # Passcode to use robot, comes as part of the constructor
        self.passcode = passcode

        # Image count
        self.num_count = 1

        # Zero approximation
        self.zero = 1e-31

        # Image arrays
        self.top_images = []
        self.right_images = []
        self.left_images = []

        # Potentially remove any images from a previous session
        #self.send_command( self.make_Image_command(-1) )

        # Measurements for the robot
        self.base_to_waist = (303. - 223.)#/1000.
        self.waist_to_shoulder = 223.#/1000.
        self.shoulder_to_elbow = 250.#/1000.
        self.elbow_to_wrist = (500. - 250.)#/1000.
        self.wrist_to_hand = (532. - 500.)#/1000.
        self.hand_length = 271.#/1000.

        # Measurements for the object
        self.x1 = 0
        self.y1 = 0
        self.x2 = 100
        self.y2 = 100

        # Offset for the end effector
        self.distance = 300/100

        # Determine the paths that the end effector needs to take
        self.top_path = []
        self.determine_paths()

    def determine_paths(self):
        m = (self.y2 - self.y1) / (self.x2 - self.x1)
        b = self.y2 - (m * self.x2)

        by = 10

        # Top
        for i in range(self.x2 - self.x1, by):
            x = self.x1 + i
            y = (m * x) + b
            z = 0
            hand = 0
            wrist = m.pi/2
            path = [hand, wrist, x, y, z]
            self.top_patha.append(path)

    # taken from https://stackoverflow.com/questions/6735917/redirecting-stdout-to-nothing-in-python
    @contextmanager
    def silence_stdout(self):
        new_target = open(os.devnull, "w")
        old_target, sys.stdout = sys.stdout, new_target
        try:
            yield new_target
        finally:
            sys.stdout = old_target

    def make_WS_command(self, hand, wrist, x, y, z):
        output = self.output
        output += hand + self.space
        output += wrist + self.space
        output += x + self.space
        output += y + self.space
        output += z + self.space
        output += "TMOVETO"

        return output

    def make_CS_command(self, hand, wrist, elbow, shoulder, waist):
        output = self.output
        output += hand + self.space
        output += wrist + self.space
        output += elbow + self.space
        output += shoulder + self.space
        output += waist + self.space
        output += "AJMA"

        return output

    def make_Image_command(self, count):
        output = str(count) + self.space + "CAPTURE"

        return output

    def parse_output(self, output):
        assert("0: You are not allowed at this time." not in output), "This is not a correct time slot"

    def send_command(self, command):
        # Send the GET request to the server
        output = urllib2.urlopen("http://debatedecide.fit.edu/robot.php?o=369&m=Y&p=" + self.passcode + "&c=" + command).read()

        # Perform error checking on the returned output
        self.parse_output(output)

    def relative_angle(self, alpha, beta, gamma):
        """
        num1_1 = m.tan(alpha) / m.sqrt(1 + m.tan(alpha)**2)
        num1_2 = self.elbow_to_wrist * m.sin(beta + gamma) * m.sin(alpha)
        num2_1 = 1 / m.sqrt(1 + m.tan(alpha)**2)
        num2_2 = self.elbow_to_wrist * m.sin(beta + gamma) * m.cos(alpha)
        num = (num1_1 * num1_2) + (num2_1 * num2_2)

        den = self.elbow_to_wrist**2
        """
        num = m.sqrt(self.elbow_to_wrist**2 * m.sin(beta + gamma)**2 * (1 + m.tan(alpha)**2))

        den1 = self.elbow_to_wrist**2 * m.sin(beta + gamma)**2 * m.sqrt(1 + m.tan(alpha)**2)
        den2 = self.elbow_to_wrist**2 * m.cos(beta + gamma)**2
        den = den1 + den2

        check = num / den
        if m.fabs(check) < 1:
            return m.acos(check)
        else:
            return None

    def FK(self, hand, wrist, elbow, shoulder, waist):
        if waist == 0.0: waist = self.zero

        h = -waist

        y0 = 1 / m.sqrt(1 + m.tan(waist)**2)
        y1 = self.shoulder_to_elbow * m.sin(shoulder)
        y2 = self.elbow_to_wrist * m.sin(shoulder+elbow)
        y3 = (self.wrist_to_hand + self.hand_length) * m.sin(shoulder+elbow+wrist)
        y = y0 * (y1 + y2 + y3)

        x = m.tan(waist) * y

        z0 = self.waist_to_shoulder
        z1 = self.shoulder_to_elbow * m.cos(shoulder)
        z2 = self.elbow_to_wrist * m.cos(shoulder+elbow)
        z3 = (self.wrist_to_hand + self.hand_length) * m.cos(shoulder+elbow+wrist)
        z = z1 + z2 + z3

        relative_angle = self.relative_angle(waist, shoulder, elbow)
        if relative_angle is not None:
            theta = wrist + relative_angle

            return [h, theta, x, y, z]
        else:
            return None

    def IK(self, hand, wrist, x, y, z):
        if y == 0: y = self.zero

        #waist = m.atan2(x,y)
        waist = m.atan2(y,x)

        xd = x - ((self.wrist_to_hand + self.hand_length) * m.cos(wrist) * m.sin(waist))
        yd = y - ((self.wrist_to_hand + self.hand_length) * m.cos(wrist) * m.cos(waist))
        zd = z + ((self.wrist_to_hand + self.hand_length) * m.sin(wrist))

        l1 = m.sqrt(xd**2 + yd**2 + (zd - self.waist_to_shoulder)**2)
        l2 = m.sqrt(xd**2 + yd**2 + zd**2)
        l3 = m.sqrt(x**2 + y**2 + z**2)

        e1 = l1**2 - self.shoulder_to_elbow**2 - self.elbow_to_wrist**2
        e2 = 2 * self.shoulder_to_elbow * self.elbow_to_wrist
        check = e1 / e2
        if m.fabs(check) <= 1:
            elbow = m.acos(check)
        else:
            return None

        g1_1 = self.shoulder_to_elbow**2 + l1**2 - self.elbow_to_wrist**2
        g1_2 = 2 * self.shoulder_to_elbow * l1
        check = g1_1 / g1_2
        if m.fabs(check) <= 1:
            g1 = m.acos(g1_1 / g1_2)
        else:
            return None

        g2_1 = self.waist_to_shoulder**2 + l1**2 - l2**2
        g2_2 = 2 * self.waist_to_shoulder * l1
        check = g2_1 / g2_2
        if m.fabs(check) < 1:
            g2 = m.acos(g2_1 / g2_2)
        else:
            return None

        shoulder = m.pi - g1 - g2


        g3 = m.pi - g1 - (m.pi - elbow)

        g4_1 = l1**2 + l2**2 - self.waist_to_shoulder**2
        g4_2 = 2 * l1 * l2
        check = g4_1 / g4_2
        if m.fabs(check) < 1:
            g4 = m.acos(g4_1 / g4_2)
        else:
            return None

        g5_1 = l2**2 + (self.wrist_to_hand + self.hand_length)**2 - l3**2
        g5_2 = 2 * l2 * (self.wrist_to_hand + self.hand_length)
        check = g5_1 / g5_2
        if m.fabs(check) < 1:
            g5 = m.acos(g5_1 / g5_2)
        else:
            return None

        """
        theta = m.pi - g3 - g4 - g5
        """
        relative_angle = self.relative_angle(waist, shoulder, elbow)
        if relative_angle is not None:
            theta = wrist - relative_angle
        else:
            return None


        h = -waist

        return [h, theta, elbow, shoulder, waist]

    def retrieve_image(self, side):
        # Capture the image
        self.send_command( self.make_Image_command(self.num_count) )

        # Form the url
        url =  "http://debatedecide.fit.edu/robot/" + str(self.num_count) + ".bmp"
        if side == "top":
            output = self.top_dir
        elif side == "right":
            output = self.right_dir
        elif side == "left":
            output = self.left_dir
        output += str(self.num_count) + ".bmp"

        # Download the file
        with self.silence_stdout():
            pass
            #wget.download(url, out=output)

        self.num_count += 1
        # Open the image
        image = cv2.imread(output)

        # Store the image if it was correctly captured, downloaded, and loaded otherwise delete it
        if type(image) == np.ndarray:
            if side == "top":
                self.top_images.append(image)
            elif side == "right":
                self.right_images.append(image)
            elif side == "left":
                self.left_images.append(image)
        else:
            os.remove(output)

    def paul_ik(self, hand, wrist, x, y, z):
        arbit_l = 113

        wrist *= m.pi / 180

        l = m.sqrt(x**2 + y**2) / 10.

        xp = l - arbit_l * m.cos(wrist)
        yp = z - arbit_l * m.sin(wrist)

        lam = m.atan2(-xp, -yp)

        shoulder = lam + m.acos((-(xp**2 + yp**2 + 250**2  - 250**2)) / (2 * 250 * m.sqrt(xp**2 + yp**2)))

        elbow = m.atan2((xp - 250 * m.cos(shoulder))/250, (yp - 250 * m.sin(shoulder)) / 250) - shoulder

        wrist_out = wrist - (shoulder + elbow)

        #waist = m.atan2(y, x)
        waist = m.atan2(x,y)

        return waist, shoulder, elbow, wrist_out

if __name__ == "__main__":
    controller = R12_Controller(michaels_code)

    """
    hand = 0.0
    wrist = m.pi/2
    x = -4650#-9.93/100
    y = -50#-43.01/100
    z = -150#.16#-16.06/100

    print("------------------------")
    print("Input")
    print("wrist = " + str(wrist))
    print("x = " + str(x))
    print("y = " + str(y))
    print("z = " + str(z))

    IK_output = controller.IK(hand, wrist, x, y, z)

    if IK_output is not None:
        print("-----------------------")
        print("IK")
        print("Wrist = " + str(IK_output[1]))
        print("Elbow = " + str(IK_output[2]))
        print("Shoulder = " + str(IK_output[3]))
        print("Waist = " + str(IK_output[4]))


        FK_output = controller.FK(IK_output[0], IK_output[1], IK_output[2], IK_output[3], IK_output[4])

        if FK_output is not None:
            print("-----------------------")
            print("FK")
            print("W = " + str(FK_output[1]))
            print("X = " + str(FK_output[2]))
            print("Y = " + str(FK_output[3]))
            print("Z = " + str(FK_output[4]))

        else:
            print("-----------------------")
            print("No FK")
    else:
        print("-----------------------")
        print("No IK")
    """

    x = -4650
    y = -50
    z = -150
    phi = 0
    waist, shoulder, elbow, wrist = controller.paul_ik(0, phi, x, y, z)

    FK_output = controller.FK(-waist, wrist, elbow, shoulder, waist)

    if FK_output is not None:
        print("-----------------------")
        print("FK")
        print("W = " + str(FK_output[1]))
        print("X = " + str(FK_output[2]))
        print("Y = " + str(FK_output[3]))
        print("Z = " + str(FK_output[4]))
