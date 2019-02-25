#!/usr/bin/env python

import rospy

# import ros message
from std_msgs.msg import Int8 # subscription
from robofriend.msg import IOWarriorData # publisher

class ServoCamDataHandler(object):
    def __init__(self, pub):
        self._pub = pub
        self._diff = 0
        self._pos = 140

    def process_data(self, data):
        self._diff = data.data
        rospy.logdebug("{%s} - Received Data: %s", self.__class__.__name__, data.data)
        self._diff_calc()

    def _diff_calc(self):
    	var = self._pos + self._diff
        if 10 <= var <= 150:
            self._pos += self._diff
            self._publish_to_iowarrior(self._pos)
        else:
            rospy.logwarn("{%s} - Position for the camera too high/low: %d", self.__class__.__name__, var)

    def _publish_to_iowarrior(self, pos = 0):
        msg = IOWarriorData()
        # msg.rgb = []
        msg.cam_pos = pos
        self._pub.publish(msg)
        rospy.logdebug("{%s} - Publish message to IOWarrior node: %s", self.__class__.__name__, msg)

def shutdown():
    rospy.signal_shutdown("Stopping Servo Cam Handler node!")

def ServoCam():
    rospy.init_node("robofriend_servo_cam_handler", log_level = rospy.INFO)
    rospy.loginfo("Starting Servo Cam Handler node!")

    pub = rospy.Publisher('/robofriend/io_warrior_data', IOWarriorData, queue_size = 1)

    dh = ServoCamDataHandler(pub)
    
    rospy.Subscriber("/robofriend/servo_cam_data", Int8, dh.process_data)

    rospy.spin()

if __name__ == '__main__':
    try:
    	ServoCam()
    except rospy.ROSInterruptException:
    	pass
