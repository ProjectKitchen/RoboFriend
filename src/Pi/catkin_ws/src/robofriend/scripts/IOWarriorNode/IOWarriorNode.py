#!/usr/bin/env python
import rospy
import os
from os.path import expanduser
import time

# import ros message
from robofriend.msg import IOWarriorData

# import ros modules
from robofriend.msg import IOWarriorData

class IOWarriorDataHandler():

    def __init__(self):
        self.__red = 255
        self.__green = 10
        self.__blue = 0
        self.__cam_pos = 140
        self._old_time = 0
        self._valid_time = 1

        self.__send_to_iowarrior(self.__red, self.__green, self.__blue, self.__cam_pos)
	#rospy.logwarn("IOWarrior init done")

    def process_data(self, data):
        rospy.logdebug("{%s} - Received message: %s, %s\n",
            self.__class__.__name__, str(data.rgb), str(data.cam_pos))

        start_time = 0
        if data.rgb:
            self.__red, self.__green, self.__blue = data.rgb
        if data.cam_pos:
            self.__cam_pos = data.cam_pos

        if self.__time_request() - self._old_time > self._valid_time:
            self.__send_to_iowarrior(self.__red, self.__green, self.__blue, self.__cam_pos)
            self._old_time = self.__time_request()
        else:
            rospy.logwarn("{%s} - Command to IOWarrior less then defined Value!",
                rospy.get_caller_id())

    def __send_to_iowarrior(self, red = 0, green = 0, blue = 0, cam_pos = 0):
	home = expanduser("~")
	cmd = "sudo " + home  + "/Git/RoboFriend/src/Pi/iowarrior/iow "
        cmd =  cmd + str(int(round(red))) + ' ' + str(int(round(green))) + ' ' + str(int(round(blue)))
	print(cmd)
	#cmd = "sudo .~/Git/RoboFriend/src/Pi/iowarrior/iow " + str(int(round(red))) + ' ' + str(int(round(green))) + ' ' + str(int(round(blue)))
        if cam_pos:
            cmd = cmd + ' ' + str(cam_pos)
        rospy.logdebug("{%s} - CMD command for IOWarrior: %s",
            self.__class__.__name__, cmd)
        os.system(cmd)

    def __time_request(self):
        return time.time()

def IOWarrior():
    rospy.init_node("robofriend_io_warrior", log_level = rospy.INFO)
    rospy.loginfo("{%s} - starting iowarrior node!",
        rospy.get_caller_id())

    iowarrior = IOWarriorDataHandler()
    rospy.Subscriber('/robofriend/io_warrior_data', IOWarriorData, iowarrior.process_data)

    rospy.spin()

if __name__ == '__main__':
    try:
        IOWarrior()
    except rospy.ROSInterruptException:
        pass
