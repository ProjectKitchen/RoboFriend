#!/usr/bin/env python

import os, sys, rospy
import serial

path = os.path.dirname(os.path.abspath(__file__)) + "/.."
sys.path.append(path)
import constants

# import ros service
from robofriend.srv import SrvPCBSensorData
from robofriend.srv import SrvRFIDData

# import ros messages
from robofriend.msg import TeensyMotorData

# import ros modules
from RFIDReader import *
from Teensy import *

def shutdown():
    rospy.signal_shutdown("Stopping Teensy Handler node!")


def SerialNode():
    rospy.init_node("robofriend_serial_data_handler", log_level = rospy.INFO)
    rospy.loginfo("Starting Serial Data Handler node!")

    tennsy_ser = None # ZAHEDIM: close the serial somewhere

    try:
        tennsy_ser = serial.Serial(constants.SER_DEV_TEENSY, constants.SER_DEV_TEENSY_BD, timeout = 1)
        rospy.loginfo("Serial for Teensy opened!")
    except Exception as inst:
        rospy.logwarn('This is a controlled catch!')
        rospy.logwarn('Serial for Teensy could not opened!')
        rospy.logwarn('Exception type: %s', type(inst))
        rospy.logwarn('Exception argument: %s', inst.args[1])

    rfid_ser = None # ZAHEDIM: close the serial somewhere
    try:
        rfid_ser = serial.Serial(constants.SER_DEV_RFID, constants.SER_DEV_RFID_BD, timeout = 1)
        rospy.loginfo("Serial for RFID Reader opened!")
    except Exception as inst:
        rospy.logwarn('This is a controlled catch!')
        rospy.logwarn('Serial for RFID reader could not opened!')
        rospy.logwarn('Exception type: %s', type(inst))
        rospy.logwarn('Exception argument: %s', inst.args[1])

    teensy_dh = Teensy(tennsy_ser)
    rfid_dh = RFIDReader(rfid_ser)

    # declare services
    serv = rospy.Service('/robofriend/get_pcb_sensor_data', SrvPCBSensorData, teensy_dh.service_handler)
    serv = rospy.Service('/robofriend/get_rfid_number', SrvRFIDData, rfid_dh.service_handler)

    # declare Subscriber callback
    rospy.Subscriber("T_TEENSY_MOTOR_DATA", TeensyMotorData, teensy_dh.motor_process_data)
    rospy.spin()
    
if __name__ == '__main__':
    try:
        SerialNode()
    except rospy.ROSInterruptException:
        pass
