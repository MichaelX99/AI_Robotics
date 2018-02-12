import urllib2
import wget
import cv2
import os
import shutil
import numpy as np
import sys
from contextlib import contextmanager


michaels_code = "PKPMWTlhvllf"

class R12_Controller:
    def __init__(self, passcode=None):
        # Make sure a passcore was provided
        try:
            assert passcode != None
        except AssertionError, e:
            print("Please enter passcode into constructor to use the R12 arm")
            raise SystemExit

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

        # Image arrays
        self.top_images = []
        self.right_images = []
        self.left_images = []

        # Potentially remove any images from a previous session
        self.send_command( self.make_Image_command(-1) )

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
        pass

    def send_command(self, command):
        # Send the GET request to the server
        output = urllib2.urlopen("http://debatedecide.fit.edu/robot.php?o=369&m=Y&p=" + self.passcode + "&c=" + command).read()

        print(output)

        # Perform error checking on the returned output
        self.parse_output(output)

    def FK(self, hand, wrist, elbow, shoulder, waist):
        pass

    def IK(self, hand, wrist, x, y, z):
        pass

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
            wget.download(url, out=output)

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

if __name__ == "__main__":
    controller = R12_Controller(michaels_code)

    controller.retrieve_image("top")
    controller.retrieve_image("right")
    controller.retrieve_image("left")