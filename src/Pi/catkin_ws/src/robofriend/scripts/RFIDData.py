#!/usr/bin/env python
import rospy

# import ros messages
from std_msgs.msg import String

# import ros services
from robofriend.srv import SrvRFIDData

# import modules
import GameCommunicator

def publishRFIDNumber(pub, args):
    if args is None:
        return 
    
    # send to game communicator
    # GameCommunicator.sendToGUI("rfid;" + str(args.rfid_number))

    data = String()
    data.data = args.rfid_number
    
    rospy.logdebug("{%s} - rfid data:\n%s", rospy.get_caller_id(), data)
    
    # publish message
    pub.publish(data)
    

def shutdown():
    rospy.loginfo("{%s} - stopping rfid data handler node.", rospy.get_caller_id())
    rospy.signal_shutdown("controlled shutdown.")

def RFIDData():
    rospy.init_node("robofriend_rfid_data", log_level = rospy.INFO)
    rospy.loginfo("{%s} - Starting rfid data handler node.", rospy.get_caller_id())
    rospy.on_shutdown(shutdown)
    
    pub = rospy.Publisher("/robofriend/rfid_number", String, queue_size = 1)
    
    rate = rospy.Rate(1) # 1hz    
    
    while not rospy.is_shutdown():
        srv_resp = None
        rospy.wait_for_service('/robofriend/get_rfid_number')
        
        try:
            request = rospy.ServiceProxy('/robofriend/get_rfid_number', SrvRFIDData)
            srv_resp = request(True)
        except rospy.ServiceException:
            rospy.logwarn("{%s} - service call failed.", rospy.get_caller_id())
   
        publishRFIDNumber(pub, srv_resp)
    
        rate.sleep() # make sure the publish rate maintains at the needed frequency

if __name__ == '__main__':
    try:
        RFIDData()
    except rospy.ROSInterruptException:
        pass