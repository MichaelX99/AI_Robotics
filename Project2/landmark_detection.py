from naoqi import ALProxy

IP = "192.168.1.5"
PORT = 9559
# Create a proxy to ALLandMarkDetection
markProxy = ALProxy("ALLandMarkDetection", IP, PORT)
# Subscribe to the ALLandMarkDetection extractor
period = 500
markProxy.subscribe("Test_Mark", period, 0.0 )


# Create a proxy to ALMemory.
memProxy = ALProxy("ALMemory", IP, PORT)
# Get data from landmark detection (assuming face detection has been activated).
data = memProxy.getData("LandmarkDetected")

print(data)
