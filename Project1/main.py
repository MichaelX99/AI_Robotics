import urllib2
import wget

michaels_code = "PKPMWTlhvllf"

class R12_Controller:
    def __init__(self, passcode):
        self.space =  "%20"
        self.output = "HOME" + self.space

        self.passcode = passcode

        self.num_count = 0

        self.send_command( self.make_Image_command(-1) )

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

if __name__ == "__main__":
    controller = R12_Controller(michaels_code)
