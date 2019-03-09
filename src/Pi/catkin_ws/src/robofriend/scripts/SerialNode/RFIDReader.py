import rospy

# import ros services
from robofriend.srv import SrvRFIDDataResponse

class RFIDReader(object):
    
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
                    """ 
                    in the old logic, the rdif number is provided to gui in a thread
                    so we should either publish the variable or make the service 
                    request by gameCommunicator in a loop 
                    """
                    readRFIDnumber = "empty"      

            except Exception as inst:                
                rospy.logwarn('This is a controlled catch!')
                rospy.logwarn('*** Read serial for RFID Handler failed! ***')
                rospy.logwarn('Exception type: %s', type(inst))
                rospy.logwarn('Exception argument: %s', inst.args[1])
        else:
            readRFIDnumber = "readRFIDnumber:01"

        rospy.loginfo("{%s} - Service handler RFID message: %s", self.__class__.__name__, readRFIDnumber)

        return SrvRFIDDataResponse(readRFIDnumber)
