#!/usr/bin/env python
import rospy, serial

# import user modules
import constants

# import ros services
from robofriend.srv import SrvRFIDData
from robofriend.srv import SrvRFIDDataResponse

# globals
ser = None

def serviceHandler(req):
    data = ""
    readRFIDnumber = None
    
    if ser is not None:
        try:
            data = str(ser.read(16))
            data = data.strip("b'")
            data = data.replace("\x02", "")
            data = data.replace("\x03", "")
            data = data.replace("\x0a", "")
            data = data.replace("\x0d", "")
            data = data.replace("\\r\\n", "")
            readRFIDnumber = data
        except Exception as inst:                
            rospy.logwarn('{%s} - this is a controlled catch.', rospy.get_caller_id())
            rospy.logwarn('{%s} - read serial for rfid handler failed.', rospy.get_caller_id())
            rospy.logwarn('{%s} - exception type: %s', rospy.get_caller_id(), type(inst))
            rospy.logwarn('{%s} - exception argument: %s', rospy.get_caller_id(), inst.args[1])
    else:
        if constants.DEBUG is True:
            readRFIDnumber = "dummy_rfid_number;01"
    
    if readRFIDnumber is not None: 
        rospy.logdebug("{%s} - service handler rfid message: %s", rospy.get_caller_id(), readRFIDnumber)
    return SrvRFIDDataResponse(readRFIDnumber)

def shutdown():
    if ser is not None:
        ser.close()
    rospy.loginfo("{%s} - stopping rfid reader node.", rospy.get_caller_id())
    rospy.signal_shutdown("controlled shutdown.")

def RFIDReader():
    global ser
    
    rospy.init_node("robofriend_rfid_reader", log_level = rospy.INFO)
    rospy.loginfo("{%s} - starting rfid reader node.", rospy.get_caller_id())
    rospy.on_shutdown(shutdown)

    try:
        ser = serial.Serial(constants.SER_DEV_RFID, constants.SER_DEV_RFID_BD)
        rospy.loginfo("{%s} - serial for rfid reader opened.")
    except Exception as inst:
        rospy.logwarn('{%s} - this is a controlled catch.', rospy.get_caller_id())
        rospy.logwarn('{%s} - serial for rfid reader could not opened.', rospy.get_caller_id())
        rospy.logwarn('{%s} - exception type: %s', rospy.get_caller_id(), type(inst))
        rospy.logwarn('{%s} - exception argument: %s', rospy.get_caller_id(), inst.args[1])

    # declare services
    rospy.Service('/robofriend/get_rfid_number', SrvRFIDData, serviceHandler)

    rospy.spin()
    
if __name__ == '__main__':
    try:
        RFIDReader()
    except rospy.ROSInterruptException:
        pass