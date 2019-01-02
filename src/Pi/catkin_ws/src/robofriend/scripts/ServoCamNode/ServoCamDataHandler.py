# import external modules
import rospy

# import ros message
from ros_robofriend.msg import IOWarriorData
from std_msgs.msg import Int8


class ServoCamDataHandler():

    def __init__(self):
        self.__cam_pos = 140
        self.__diff = 0

        self.__iowarrior_pub = rospy.Publisher('T_IOWARRIOR_DATA', IOWarriorData, queue_size = 10)
        self.__iowarrior_msg = IOWarriorData()

    def process_data(self, data):
        self.__diff = data.data
        print("[INFO] {} - Received Data: {}"
                .format(self.__class__.__name__, data.data))
        self.__diff_calc()

    def __diff_calc(self):
        if 10 <= self.__cam_pos + self.__diff <= 150:
            self.__cam_pos += self.__diff
            self.__send_to_iowarrior(self.__cam_pos)
        else:
            print("[INFO] Position for the camera too high/low: {}\n".format(self.__cam_pos + self.__diff))

    def __send_to_iowarrior(self, cam_position):
        self.__iowarrior_msg.rgb = []
        self.__iowarrior_msg.cam_pos = cam_position
        self.__iowarrior_pub.publish(self.__iowarrior_msg)
        print("[INFO] {} - Publish message to IOWarrior Node: {}\n".format(self.__class__.__name__, self.__iowarrior_msg))
