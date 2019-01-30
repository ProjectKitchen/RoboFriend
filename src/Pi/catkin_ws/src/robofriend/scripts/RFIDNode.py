#!/usr/bin/env python

import rospy
import serial
import constants

# import ros services
from robofriend.srv import SrvRFIDData, SrvRFIDDataResponse

class RFIDDataHandler(object):
    def __init__(self, serial):
        self._serial = serial

    def service_handler(self, request):
        data = ""
        readRFIDnumber = None

        if self._serial is not None:
            try:
                data = str(self._serial.read(16))
                data = data.strip("b'")
                data = data.replace("\\x02", "").replace("\\x03", "").replace("\\x0a", "").replace("\\x0d", "").replace("\\r\\n", "")
                
                readRFIDnumber = data
                if readRFIDnumber != "empty":
                    readRFIDnumber = "empty"      

            except Exception as e:
                rospy.logwarn('*** Read serial for RFID Handler failed! ***')
                rospy.logwarn(type(inst))
                rospy.logwarn(inst.args)
        else:
            readRFIDnumber = "readRFIDnumber:01"

        rospy.loginfo("{%s} - Service handler RFID message: %s", self.__class__.__name__, readRFIDnumber)

        return SrvRFIDDataResponse(readRFIDnumber)

def shutdown():
    rospy.signal_shutdown("Stopping RFID node!")

def RFID():
    rospy.init_node("robofriend_rfid_handler", log_level = rospy.INFO)
    rospy.loginfo("Starting RFID Handler node!")
    
    ser = None

    try:
        ser = serial.Serial(constants.SER_DEV_RFID, constants.SER_DEV_RFID_BD)
        rospy.loginfo("Serial for RFID reader opened!")

    except Exception as inst:
        rospy.logwarn('This is a controlled catch!')
        rospy.logwarn('Serial for FRID reader could not opened!')
        rospy.logwarn('Exception type: %s', type(inst))
        rospy.logwarn('Exception argument: %s', inst.args[1])

    dh = RFIDDataHandler(ser)

    # declare service
    serv = rospy.Service('/robofriend/get_rfid_data', SrvRFIDData, dh.service_handler)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        RFID()
    except rospy.ROSInterruptException:
        pass