#!/usr/bin/env python

import rospy

# import ros message
from std_msgs.msg import Int8
from robofriend.msg import IOWarriorData

class ServoCamDataHandler(object):
    def __init__(self, pub):
        self._pub = pub
        self._diff = 0
        self._cam_pos = 140

    def process_data(self, data):
        # TOOD: process rgb data
        self._diff = data.data
        rospy.logdebug("{%s} - Received Data: %s", self.__class__.__name__, data.data)
        self._diff_calc()

    def _diff_calc(self):
    	var = self._cam_pos + self._diff
        if 10 <= var <= 150:
            self._cam_pos += self._diff
            self._send_to_iowarrior(pos = self._cam_pos)
        else:
            rospy.logwarn("{%s} - Position for the camera too high/low: %d", self.__class__.__name__, var)

    def _send_to_iowarrior(self, r = 255, g = 255, b = 255, pos = 0):
        msg = IOWarriorData()
        msg.rgb = [r, g, b]
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
    
    # TODO: subscribe to rgb data 
    rospy.Subscriber("/robofriend/servo_cam_data", Int8, dh.process_data)

    rospy.spin()

if __name__ == '__main__':
    try:
    	ServoCam()
    except rospy.ROSInterruptException:
    	pass