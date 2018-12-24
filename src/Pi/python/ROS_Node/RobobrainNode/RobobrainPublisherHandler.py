import rospy

#import ros message
from ros_robofriend.msg import SpeechData
from ros_robofriend.msg import TeensyMotorData

class RobobrainPublisherHandler():
    
    def __init__(self, topics):
        self._topics = topics
        self._speech_pub = rospy.Publisher(topics['T_SPEECH_DATA'], SpeechData, queue_size = 10)
        self._teensy_motor_pub = rospy.Publisher(topics['T_TEENSY_MOTOR_DATA'], TeensyMotorData, queue_size = 10)
        self._speech_msg = SpeechData()
        self._teensy_motor_msg = TeensyMotorData()

    def speech_message_publish(self, mode, text = None):
        self._speech_msg.mode = mode
        self._speech_msg.text = text
        self._speech_pub.publish(self._speech_msg)
        print("[INFO] {} - Speech Published Data: {}".format(self.__class__.__name__, self._speech_msg))

    def teensy_motor_message_publish(self, direction, duration):
        self._teensy_motor_msg.duration = duration
        self._teensy_motor_msg.direction = direction
        self._teensy_motor_pub.publish(self._teensy_motor_msg)
        print("[INFO] {} - Teensy Motor Published Data: {}\n".format(self.__class__.__name__, self._teensy_motor_msg))
