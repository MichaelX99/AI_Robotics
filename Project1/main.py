import urllib2

def make_WS_command(hand, wrist, x, y, z):
    space = "%20"
    output = "HOME" + space
    output += hand + space
    output += wrist + space
    output += x + space
    output += y + space
    output += z + space
    output += "TMOVETO"

    return output

def make_CS_command(hand, wrist, elbow, shoulder, waist):
    space = "%20"
    output = "HOME" + space
    output += hand + space
    output += wrist + space
    output += elbow + space
    output += shoulder + space
    output += waist + space
    output += "AJMA"

    return output

def parse_output(output):
    pass

def send_command(command):
    # Send the GET request to the server
    output = urllib2.urlopen("http://debatedecide.fit.edu/robot.php?o=369&m=Y&p=PKPMWTlhvllf&c="+command).read()

    # Perform error checking on the returned output
    parse_output(output)

def FK(hand, wrist, elbow, shoulder, waist):
    pass

def IK(hand, wrist, x, y, z):
    pass
