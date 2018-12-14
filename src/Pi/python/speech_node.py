import pyttsx3
import random
import time
import threading
import statusModule
import SpeechDataHandler
import rospy

# import ros message_publish
from ros_robofriend.msg import SpeechData

def node_start():
    print("[INFO] ROS Speech Node started!\n")

    # thread to handle incoming messages
    speech_thread = threading.Thread(
        target = handle_speech
    )

    # start thread as daemon
    speech_thread.daemon = True

    # start speech thread
    speech_thread.start()

def handle_speech():

    lastSpeakTimestamp = time.time()
    lastSpeakWord = None
    wordRate = 140                  # words per minute
    volumeRate = 1.0
    language = 'german'

    speech_engine_dict = {
        "language" : language, \
        "word_rate" : wordRate, \
        "volume_rate" : volumeRate
    }

    speech_engine = InitSpeechEngine(speech_engine_dict)

    speech = SpeechDataHandler.SpeechDataHandler(speech_engine, speech_engine_dict['language'])
    rospy.Subscriber("T_SPEECH_DATA", SpeechData, speech.process_data)

def InitSpeechEngine(speech_engine_dict):
    speechEngine = pyttsx3.init(debug=True)
    speechEngine.setProperty('rate', speech_engine_dict['word_rate'])
    speechEngine.setProperty('volume', speech_engine_dict['volume_rate'])
    speechEngine.setProperty('voice', speech_engine_dict['language'])
    return speechEngine
