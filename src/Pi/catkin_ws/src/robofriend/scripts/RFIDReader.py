import rospy

# import user modules
import constants

# import ros services
from robofriend.srv import SrvRFIDDataResponse

class RFIDReader(object):
    
    def __init__(self, serial):
        self._serial = serial

    def service_handler(self, req):
        data = ""
        readRFIDnumber = None

        if self._serial is not None:
            try:
                data = str(self._serial.read(16))
                data = data.strip("b'")
                data = data.replace("\x02", "")
                data = data.replace("\x03", "")
                data = data.replace("\x0a", "")
                data = data.replace("\x0d", "")
                data = data.replace("\\r\\n", "")
                readRFIDnumber = data
            except Exception as inst:                
                rospy.logwarn('this is a controlled catch.')
                rospy.logwarn('*** read serial for rfid handler failed. ***')
                rospy.logwarn('exception type: %s', type(inst))
                rospy.logwarn('exception argument: %s', inst.args[1])
        else:
            if constants.DEBUG is True:
                readRFIDnumber = "dummy_rfid_number;01"
            
        rospy.loginfo("{%s} - service handler rfid message: %s", rospy.get_caller_id(), readRFIDnumber)

        return SrvRFIDDataResponse(readRFIDnumber)