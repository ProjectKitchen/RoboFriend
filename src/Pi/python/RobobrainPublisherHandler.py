import rospy

#import ros message
from ros_robofriend.msg import SpeechData

class RobobrainPublisherHandler():
    def __init__(self):
        self._speech_pub = rospy.Publisher('T_SPEECH_DATA', SpeechData, queue_size = 10)
        self._speech_msg = SpeechData()

    def speech_message_publish(self, mode, text = None):
        self._speech_msg.mode = mode
        self._speech_msg.text = text
        self._speech_pub.publish(self._speech_msg)
        print("[INFO] {} - Speech Published Data: {}".format(self.__class__.__name__, self._speech_msg))
