# import external modules
import rospy

# import ros message
from std_msgs.msg import Int8

class ServoCamDataHandler()

    def __init__(self):
        self.__cam_pos = 140
        self.__diff = 0

    def process_data(self, data):
        self.__diff = data.Int8
        print("[INFO] {} - Received Data: {}"
                .format(self.__class__.__name__, data.Int8))
