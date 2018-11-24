import pyttsx3
import random
import time
import threading
import statusModule

# global variables
lastSpeakTimestamp = time.time()
lastSpeakWord = None
wordRate = 140
speechEngne =
runFlag = True


speech_mode = {
    "custom"    : custom_speech,
    "random"    : random_speech,
    "bullshit"  : bullshit_speech
}

def callback(data):
    node = None
    mode = None
    message = None

    rospy.loginfo("[INFO] Received message: {}".format(data))

    try:
        node, mode, message = data.split(";")
    except ValueError:
        node, mode = data.split(";")

    speech_mode[mode](message)

def node_start():
    rospy.init_node("speech_node", anonymous = True)
    rospy.Subscriber("speech_node_topic", String, callback) #TODO: Define topic name in brain node


def custom_speech(speech_content):
    pass

def random_speech():
    pass

def bullshit_speech():
    pass
