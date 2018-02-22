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
        self.output = ""#"HOME" + self.space

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
        self.base_to_waist = (303. - 223.) * 100# / 1000.
        self.waist_to_shoulder = 223. * 100# / 1000.
        self.shoulder_to_elbow = 250. * 100# / 1000.
        self.elbow_to_wrist = (500. - 250.) * 100# / 1000.
        self.wrist_to_hand = (532. - 500.) * 100# / 1000.
        self.hand_length = 271. * 100# / 1000.

        # Measurements for the object
        self.x2 = -1000.
        self.y2 = -4000.
        self.x1 = -4600.
        self.y1 = -104.

        # Determine the paths that the end effector needs to take
        self.top_path = []
        self.CS_path = []
        self.WS_path = []
        self.extrude_object()
        self.determine_AJMA_path()
        self.determine_TMOVETO_path()

        for path in self.WS_path:
            print(path)

    def extrude_object(self):
        slope = (self.y2 - self.y1) / (self.x2 - self.x1)
        b = self.y1 - (slope * self.x1)

        by = 200

        r = int(m.fabs(self.x1 - self.x2))

        # Top
        for i in range(0, r+by, by):
            x = int(self.x1 + i)
            y = int((slope * x) + b)
            z = -150
            hand = 0
            wrist = 0#m.pi/2
            path = [hand, wrist, x, y, z]
            self.top_path.append(path)

    def determine_AJMA_path(self):
        for path in self.top_path:
            wrist = 0.0
            x = float(path[2])
            y = float(path[3])
            z = float(path[4])

            hand, waist, shoulder, elbow, wrist_out = self.paul_ik(float(wrist), float(x), float(y), float(z))

            hand = int(hand)
            wrist_out = int(wrist_out)
            elbow = int(elbow)
            shoulder = int(shoulder)
            waist = int(waist)

            cs_path = [hand, wrist_out, elbow, shoulder, waist]

            self.CS_path.append(cs_path)

    def determine_TMOVETO_path(self):
        x1 = -854.
        y1 = -3418.
        z1 = -1285.

        x2 = -2346.
        z2 = -1360.

        x3 = -3817.
        y3 = -86.
        z3 = -1295.

        slope = (y3 - y1) / (x3 - x1)
        b = y1 - (slope * x1)

        by = 200

        r = int(m.fabs(x1 - x3))

        xs = np.array([x1, x2, x3])
        zs = np.array([z1, z2, z3])

        # Z IS NOT STRAIGHT IN TMOVETO IT ARCS.  however it is a larger arc than a 4th degree and I do not care enough to get more points to make it more exact
        coefs = np.polyfit(xs, zs, 4)

        # Top
        for i in range(0, r+by, by):
            x = int(x1 - i)
            y = int((slope * x) + b)
            z = int((coefs[0] * x**4) + (coefs[1] * x**3) + (coefs[2] * x**2) + (coefs[3] * x) + coefs[4])
            hand = int(m.degrees(-m.atan2(-x, -y)) * 100.)
            wrist = -9000.
            path = [hand, wrist, x, y, z]
            self.WS_path.append(path)

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
        output = output + str(hand) + self.space
        output = output + str(wrist) + self.space
        output = output + str(x) + self.space
        output = output + str(y) + self.space
        output = output + str(z) + self.space
        output += "TMOVETO"

        return output

    def make_CS_command(self, hand, wrist, elbow, shoulder, waist):
        output = self.output
        output = output + str(hand) + self.space
        output = output + str(wrist) + self.space
        output = output + str(elbow) + self.space
        output = output + str(shoulder) + self.space
        output = output + str(waist) + self.space
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
        z = z0 + z1 + z2 + z3

        relative_angle = self.relative_angle(waist, shoulder, elbow)
        if relative_angle is not None:
            theta = wrist + relative_angle

            return [h, theta, x, y, z]
        else:
            return None

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

    def paul_ik(self, wrist, x, y, z):
        arbit_l = 113.

        wrist *= m.pi / 180.

        l = m.sqrt(x**2 + y**2) / 10.

        xp = l - arbit_l * m.cos(wrist)
        yp = z - arbit_l * m.sin(wrist)

        lam = m.atan2(-xp, -yp)

        shoulder = lam + m.acos((-(xp**2 + yp**2 + 250.**2  - 250.**2)) / (2. * 250. * m.sqrt(xp**2 + yp**2)))

        elbow = m.atan2((xp - 250. * m.cos(shoulder))/250., (yp - 250. * m.sin(shoulder)) / 250.) - shoulder

        wrist_out = wrist - (shoulder + elbow)

        waist = m.atan2(-x,-y)

        hand = -waist

        hand = int(100. * m.degrees(hand))
        waist = int(100. * m.degrees(waist))
        shoulder = -int(100. * m.degrees(shoulder))
        elbow = -int(100. * m.degrees(elbow))
        wrist_out = -int(100. * (m.degrees(wrist_out) + 180.))

        return hand, waist, shoulder, elbow, wrist_out

    def run(self):

        for path in self.CS_path:
            hand = path[0]
            wrist = path[1]
            elbow = path[2]
            shoulder = path[3]
            waist = path[4]

            command = self.make_CS_command(int(hand), int(wrist), int(elbow), int(shoulder), int(waist))

            self.send_command(command)
        

        for path in self.WS_path:
            hand = path[0]
            wrist = path[1]
            x = path[2]
            y = path[3]
            z = path[4]

            command = self.make_WS_command(int(hand), int(wrist), int(x), int(y), int(z))

            #self.send_command(command)

if __name__ == "__main__":
    controller = R12_Controller(michaels_code)

    controller.run()
