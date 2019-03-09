#!/usr/bin/env python
import rospy

# import ros services
from robofriend.srv import SrvRFIDData

# import ros messages
from std_msgs.msg import String

class RFIDReaderDataHandler(object):
    def __init__(self, pub):
        self._pub = pub
        self._rfid_number = "empty"
        
    def publishRFIDNumber(self, args):
        if args is None:
            return 
        
        self._rfid_number = args.rfid_number
        
        data = String()
        data.data = self._rfid_number
        
        rospy.logdebug("{%s} RFID data:\n%s", 
            self.__class__.__name__,
            data)
        
        # publish message
        self._pub.publish(data)

def shutdown():
    rospy.signal_shutdown("Stopping RFID Handler node!")

def RFIDReader():
    rospy.init_node("robofriend_rfid_handler", log_level = rospy.INFO)
    rospy.loginfo("Starting RFID Handler node!")
    
    pub = rospy.Publisher("/robofriend/rfid_number", String, queue_size = 1)
    
    dh = RFIDReaderDataHandler(pub)
    
    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        srv_resp = None
        rospy.wait_for_service('/robofriend/get_rfid_number')
        
        try:
            request = rospy.ServiceProxy('/robofriend/get_rfid_number', SrvRFIDData)
            srv_resp = request(True)
        except rospy.ServiceException:
            rospy.logwarn("{%s} - Service call failed", self.__class__.__name__)
   
        dh.publishRFIDNumber(srv_resp)
    
        rate.sleep() # make sure the publish rate maintains at the needed frequency

if __name__ == '__main__':
    try:
        RFIDReader()
    except rospy.ROSInterruptException:
        pass