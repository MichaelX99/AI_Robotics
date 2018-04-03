from naoqi import ALProxy
import qi

PORT = 9559

# Set here your robto's ip.
ROBOTIP = "192.168.1.5"

tts = ALProxy("ALTextToSpeech", ROBOTIP, PORT)
tts.say("Let me sit for a bit!")

session = qi.Session()

try:
    session.connect("tcp://" + ROBOTIP + ":" + str(PORT))
except RuntimeError:
    print ("Can't connect to Naoqi at ip \"" + ROBOTIP + "\" on port " + PORT +".\n"
           "Please check your script arguments. Run with -h option for help.")
    sys.exit(1)

posture_service = session.service("ALRobotPosture")
posture_service.goToPosture("SitRelax", 1.0)
