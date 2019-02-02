# import external modules
import os
import rospy

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
