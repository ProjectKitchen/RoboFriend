#!/usr/bin/env python
import os
import rospy

# import ros message
from robofriend.msg import IOWarriorData

def send_to_iowarrior(self, red = 0, green = 0, blue = 0, cam_pos = 0):
    cmd = "sudo ./iowarrior/iow " + str(int(round(red))) + ' ' + str(int(round(green))) + ' ' + str(int(round(blue)))
    if cam_pos:
        cmd = cmd + ' ' + str(cam_pos)
    rospy.loginfo("CMD for IOWarrior:  %s", cmd)
    os.system(cmd)

class IOWarriorDataHandler():

    def __init__(self):
        self._red = 0
        self._green = 10
        self._blue = 10
        self._cam_pos = 140

        send_to_iowarrior(
        	self._red, 
        	self._green, 
        	self._blue, 
        	self._cam_pos
        	)

    def process_data(self, data):
    	rospy.loginfo("{%s} - rgb: %s, position: %s", 
    		self.__class__.__name__,
    		data.rgb,
            data.cam_pos)

        if data.rgb:
            self._red, self._green, self._blue = data.rgb
        if data.cam_pos:
            self._cam_pos = data.cam_pos

        send_to_iowarrior(self._red, self._green, self._blue, self._cam_pos)

def shutdown():
    rospy.signal_shutdown("Stopping IOWarrior Data Handler node!")

def IOWarrior():
	rospy.init_node("robofriend_io_warrior_data", log_level = rospy.INFO)
	rospy.loginfo("Starting IOWarrior Data Handler node")

	dh = IOWarriorDataHandler()
	rospy.Subscriber("/robofriend/io_warrior_data", IOWarriorData, dh.process_data)

	rospy.spin()

if __name__ == '__main__':
    try:
        IOWarrior()
    except rospy.ROSInterruptException:
        pass