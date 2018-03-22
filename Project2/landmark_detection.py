from naoqi import ALProxy
IP = "your_robot_ip"
PORT = 9559
# Create a proxy to ALLandMarkDetection
markProxy = ALproxy("ALLandMarkDetection", IP, PORT)
# Subscribe to the ALLandMarkDetection extractor
period = 500
markProxy.subscribe("Test_Mark", period, 0.0 )


# Create a proxy to ALMemory.
memProxy = ALProxy("ALMemory", IP, PORT)
# Get data from landmark detection (assuming face detection has been activated).
data = memProxy.getData("LandmarkDetected")

print(data)
