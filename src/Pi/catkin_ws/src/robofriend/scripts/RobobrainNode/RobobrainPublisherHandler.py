import rospy

#import ros message
from ros_robofriend.msg import SpeechData
from ros_robofriend.msg import TeensyMotorData
from ros_robofriend.msg import EarsLedData
from ros_robofriend.msg import ServoCamData

class RobobrainPublisherHandler():

    def __init__(self, topics):
        self._topics = topics
        self._speech_pub = rospy.Publisher(topics['T_SPEECH_DATA'], SpeechData, queue_size = 10)
        self._teensy_motor_pub = rospy.Publisher(topics['T_TEENSY_MOTOR_DATA'], TeensyMotorData, queue_size = 10)
        self._ears_led_pub = rospy.Publisher(topics['T_EARS_LED_DATA'], EarsLedData, queue_size = 10)
        self._servo_cam_pub = rospy.Publisher(topics['T_SERVO_CAM_DATA'], ServoCamData, queue_size = 10)
        self._speech_msg = SpeechData()
        self._teensy_motor_msg = TeensyMotorData()
        self._ears_led_msg = EarsLedData()
        self._servo_cam_msg = ServoCamData()

    def speech_message_publish(self, mode, text = None):
        self._speech_msg.mode = mode
        self._speech_msg.text = text
        self._speech_pub.publish(self._speech_msg)
        print("[INFO] {} - Speech Published Data: {}\n".format(self.__class__.__name__, self._speech_msg))

    def teensy_motor_message_publish(self, direction, duration):
        self._teensy_motor_msg.duration = duration
        self._teensy_motor_msg.direction = direction
        self._teensy_motor_pub.publish(self._teensy_motor_msg)
        print("[INFO] {} - Teensy Motor Published Data: {}\n".format(self.__class__.__name__, self._teensy_motor_msg))

    def ears_led_message_publish(self, random = "", repeat_num = [0, 0], rgb_color = []):
        self._ears_led_msg.random = random
        self._ears_led_msg.repeat_num = repeat_num
        self._ears_led_msg.rgb_color = rgb_color
        self._ears_led_pub.publish(self._ears_led_msg)
        print("[INFO] {} - Ears/Led Published Data: {}\n".format(self.__class__.__name__, self._ears_led_msg))

    def servo_cam_message_publish(self, max_min = "", diff = 0):
        self._servo_cam_msg.max_min = max_min
        self._servo_cam_msg.diff = diff
        self._servo_cam_pub.publish(self._servo_cam_msg)
        print("[INFO] {} - Servo Camera Published Data: {}\n".format(self.__class__.__name__, self._servo_cam_msg))
