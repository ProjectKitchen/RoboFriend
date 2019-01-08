#!/usr/bin/env python
import rospy

# import ros message
from robofriend.msg import LedEarsData # subscription
from robofriend.msg import IOWarriorData # publisher

class LedEarsDataHandler():

    def __init__(self, pub):
        self._pub = pub
        self._random = None
        self._repeat_num = None
        self._rgb = None
        
    def process_data(self, data):
        self._random = data.random
        self._repeat_num = list(data.repeat_num)
        self._rgb = list(data.rgb_color)
        rospy.logdebug("{%s} - Received Data: %s %s %s",
            self.__class__.__name__, 
            self._random, 
            self._repeat_num, 
            self._rgb)

    def _publish_to_iowarrior(self, cam_position):
        msg = IOWarriorData()
        msg.rgb = []
        msg.cam_pos = cam_position
        self._pub.publish(msg)
        rospy.logdebug("{%s} - Publish message to IOWarrior node: %s", self.__class__.__name__, msg)

def LedEars():
    rospy.init_node("robofriend_led_ears_handler", log_level = rospy.INFO)
    rospy.loginfo("Starting LED Ears Data Handler node!")

    pub = rospy.Publisher('/robofriend/io_warrior_data', IOWarriorData, queue_size = 1)

    dh = LedEarsDataHandler(pub)

    rospy.Subscriber("/robofriend/led_ears_data", LedEarsData, dh.process_data)

    rospy.spin()

if __name__ == '__main__':
    try:
        LedEars()
    except rospy.ROSInterruptException:
        pass