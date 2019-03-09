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
                # data = data.strip("b'") # TODO: is this nessecary?
                data = data.replace("\x02", "")
                data = data.replace("\x03", "")
                data = data.replace("\x0a", "")
                data = data.replace("\x0d", "")
                # data = data.replace("\\r\\n", "")  # TODO: is this nessecary?
                # lock
                readRFIDnumber = data
                # release
                if readRFIDnumber != "empty": #if rfid not locked: if rfid != empty, lock. --- release
                    # in the old logic, the rdif number is provided to gui in a thread:
                    # gameCommunicator.sendtogui("rfid;"+str(readRFIDnumber))
                    # ZAHEDIM: time the return value and the clearance
                    readRFIDnumber = "empty" 
            except Exception as inst:                
                rospy.logwarn('This is a controlled catch!')
                rospy.logwarn('*** Read serial for RFID Handler failed! ***')
                rospy.logwarn('Exception type: %s', type(inst))
                rospy.logwarn('Exception argument: %s', inst.args[1])
        else:
            readRFIDnumber = "Dummy RFID number: 01"
            
        rospy.loginfo("{%s} - Service handler RFID message: %s", self.__class__.__name__, readRFIDnumber)

        return SrvRFIDDataResponse(readRFIDnumber)
