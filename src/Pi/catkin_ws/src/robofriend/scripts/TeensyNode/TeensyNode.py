#!/usr/bin/env python

import rospy

# import ros service
from robofriend.srv import SrvPCBSensorData

# import ros messages
from ros_robofriend.msg import TeensyMotorData

# import ros modules
from TeensyDataHandler import *

def shutdown():
    rospy.signal_shutdown("Stopping Teensy Handler node!")


def Teensy():
    rospy.init_node("robofriend_teensy_communicator")
    rospy.loginfo("Starting Teensy Handler node!")

    serial = None

    try:
        serial = serial.Serial("/dev/ttyACM0", 9600, timeout = 1)
        rospy.loginfo("*** Serial for Teensy opened! ***")
    except Exception as inst:
        rospy.logwarn('*** Serial for Teensy could not opened! ***')
        rospy.logwarn(type(inst))
        rospy.logwarn(inst.args)

    dh = TeensyDataHandler(serial)

    # declare service
    serv = rospy.Service('/robofriend/get_pcb_sensor_data', SrvPCBSensorData, dh.service_handler)

    # declare Subscriber callback
    rospy.Subscriber("T_TEENSY_MOTOR_DATA", TeensyMotorData, dh.motor_process_data)
    rospy.spin()

if __name__ == '__main__':
    try:
        Teensy()
    except rospy.ROSInterruptException:
        pass