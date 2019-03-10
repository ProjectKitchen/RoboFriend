#!/usr/bin/env python

import rospy
import serial
import constants

# import ros messages

# import ros service
from robofriend.srv import SrvPCBSensorData
from robofriend.srv import SrvRFIDData

# import modules
from RFIDReader import RFIDReader
from TeensyCommunicator import TeensyCommunicator

# globals 
ser_rfid = None
ser_teensy = None

def shutdown():
    if ser_rfid is not None:
        ser_rfid.close()
    if ser_teensy is not None:
        ser_teensy.close()
    rospy.loginfo("{%s} - stopping serial data handler node.", rospy.get_caller_id())
    rospy.signal_shutdown("controlled shutdown.")

def SerialNode():
    global ser_rfid, ser_teensy
    
    rospy.init_node("robofriend_serial_data_handler", log_level = rospy.INFO)
    rospy.loginfo("{%s} - starting serial data handler node.", rospy.get_caller_id())
    rospy.on_shutdown(shutdown)

    try:
        ser_rfid = serial.Serial(constants.SER_DEV_RFID, constants.SER_DEV_RFID_BD)
        rospy.loginfo("serial for rfid reader opened.")
    except Exception as inst:
        rospy.logwarn('this is a controlled catch.')
        rospy.logwarn('serial for rfid reader could not opened.')
        rospy.logwarn('exception type: %s', type(inst))
        rospy.logwarn('exception argument: %s', inst.args[1])

    try:
        ser_teensy = serial.Serial(constants.SER_DEV_TEENSY, constants.SER_DEV_TEENSY_BD, timeout = 1)
        rospy.loginfo("serial for teensy opened.")
    except Exception as inst:
        rospy.logwarn('this is a controlled catch.')
        rospy.logwarn('terial for teensy could not opened.')
        rospy.logwarn('exception type: %s', type(inst))
        rospy.logwarn('exception argument: %s', inst.args[1])

    rfid = RFIDReader(ser_rfid)
    teensy = TeensyCommunicator(ser_teensy)

    # declare services
    rospy.Service('/robofriend/get_rfid_number', SrvRFIDData, rfid.service_handler)
    rospy.Service('/robofriend/get_pcb_sensor_data', SrvPCBSensorData, teensy.service_handler)

    rospy.spin()
    
if __name__ == '__main__':
    try:
        SerialNode()
    except rospy.ROSInterruptException:
        pass
