from naoqi import ALProxy
import time

IP = "192.168.1.5"
PORT = 9559

speech_proxy = ALProxy("ALTextToSpeech", IP, PORT)
speech_proxy.say("Starting Application")


print("1")
al_proxy = ALProxy("ALAutonomousLife", IP, PORT)
al_proxy.setState("solitary")

print("2")



face_detection_proxy = ALProxy("ALFaceDetection", IP, PORT)
face_detection_proxy.setTrackingEnabled(False)
face_detection_proxy.setRecognitionEnabled(False)

print("3")

people_detection_proxy = ALProxy("ALPeoplePerception", IP, PORT)
people_detection_proxy.setMovementDetectionEnabled(False)

print("4")

landmark_detection_proxy = ALProxy("ALLandMarkDetection", IP, PORT)
period = 500
landmark_detection_proxy.subscribe("Test_LandMark", period, 0.0 )

print("5")

memValue = "LandmarkDetected"
memoryProxy = ALProxy("ALMemory", IP, PORT)

print("6")

for i in range(0, 40):
  time.sleep(0.5)
  val = memoryProxy.getData(memValue)

  print(i)

speech_proxy.say("Ending Application")
