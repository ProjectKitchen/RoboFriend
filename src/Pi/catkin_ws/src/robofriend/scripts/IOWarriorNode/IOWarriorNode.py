#!/usr/bin/env python
import rospy
import os

# import ros message
from robofriend.msg import IOWarriorData

# import ros modules
from robofriend.msg import IOWarriorData

class IOWarriorDataHandler():

    def __init__(self):
        self.__red = 0
        self.__green = 10
        self.__blue = 10
        self.__cam_pos = 140

        self.__send_to_iowarrior(self.__red, self.__green, self.__blue, self.__cam_pos)

    def process_data(self, data):
        rospy.logdebug("{%s} - Received message: %s, %s\n",
            self.__class__.__name__, str(data.rgb), str(data.cam_pos))
        if data.rgb:
            self.__red, self.__green, self.__blue = data.rgb
        if data.cam_pos:
            self.__cam_pos = data.cam_pos

        self.__send_to_iowarrior(self.__red, self.__green, self.__blue, self.__cam_pos)

    def __send_to_iowarrior(self, red = 0, green = 0, blue = 0, cam_pos = 0):
        cmd = "sudo ./iowarrior/iow " + str(int(round(red))) + ' ' + str(int(round(green))) + ' ' + str(int(round(blue)))
        if cam_pos:
            cmd = cmd + ' ' + str(cam_pos)
        rospy.logdebug("{%s} - CMD command for IOWarrior: %s",
            self.__class__.__name__, cmd)
        os.system(cmd)

def IOWarrior():
    rospy.init_node("robofriend_io_warrior_handler", log_level = rospy.INFO)
    rospy.loginfo("Starting IOWarrior Data Handler Node")

    iowarrior = IOWarriorDataHandler()
    rospy.Subscriber('/robofriend/io_warrior_data', IOWarriorData, iowarrior.process_data)

    rospy.spin()

if __name__ == '__main__':
    try:
        IOWarrior()
    except rospy.ROSInterruptException:
        pass
