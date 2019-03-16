#!/usr/bin/env python3
import rospy

# import ros message and services
from robofriend.msg import IOWarriorData
from robofriend.msg import ServoCamData
from robofriend.srv import SrvServoCameraData, SrvServoCameraDataResponse

def shutdown():
    rospy.loginfo("{%s} - stopping servo/cam data handler", rospy.get_caller_id())
    rospy.signal_shutdown("Stopping Servo/Cam node!")

class ServoCamDataHandler():

    def __init__(self):
        self.__cam_pos = 140
        self.__diff = 0
        self.__max_min = None

        self.__iowarrior_pub = rospy.Publisher('/robofriend/io_warrior_data', IOWarriorData, queue_size = 10)
        self.__iowarrior_msg = IOWarriorData()

        # Webserver services
        rospy.Service('/robofriend/camera_position', SrvServoCameraData, self._service_handler)

    def _service_handler(self, request):
        rospy.logdebug("{%s} - Camera Change request received!",
            rospy.get_caller_id())
        self.__diff = request.cam_change
        self.__diff_calc()

    def _process_data(self, data):
        self.__max_min = data.max_min
        self.__diff = data.diff
        rospy.logdebug("{%s} - Received data: %s",
            self.__class__.__name__, str(data))

        self.__diff_calc()

    def __diff_calc(self):
        if self.__max_min == "max":
            self.__cam_pos = 150
            self.__send_to_iowarrior(self.__cam_pos)
        elif self.__max_min == "min":
            self.__cam_pos = 10
            self.__send_to_iowarrior(self.__cam_pos)
        else:
            if 10 <= self.__cam_pos + self.__diff <= 150:
                self.__cam_pos += self.__diff
                self.__send_to_iowarrior(self.__cam_pos)
            else:
                rospy.logdebug("{%s} - Position for camera to high/low: %s",
                    rospy.get_caller_id(), str(self.__cam_pos + self.__diff))

    def __send_to_iowarrior(self, cam_position):
        self.__iowarrior_msg.rgb = []
        self.__iowarrior_msg.cam_pos = cam_position
        self.__iowarrior_pub.publish(self.__iowarrior_msg)
        rospy.logdebug("{%s} - Publish message to IOWarrior Node: {%s}",
            self.__class__.__name__, str(self.__iowarrior_msg))

def ServoCam():
    rospy.init_node("robofriend_servo_cam", log_level = rospy.DEBUG)
    rospy.loginfo("Starting Servo Cam Node!")

    servocam = ServoCamDataHandler()
    rospy.Subscriber("/robofriend/servo_cam_data",
                ServoCamData, servocam._process_data)

    rospy.spin()

if __name__ == '__main__':
    try:
        ServoCam()
    except rospy.ROSInterruptException:
        pass
