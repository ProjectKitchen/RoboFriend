#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from robofriend.msg import CamData
from robofriend.msg import IRSensorData

# import ros services
from robofriend.srv import SrvRFIDData

def Talker():
    rospy.init_node('robofriend_talker', anonymous = True, log_level = rospy.INFO)
    rospy.loginfo("Starting Talker node!")

    # pub_o = rospy.Publisher('/robofriend/odom_data', Pose, queue_size = 10) # odometry data
    # pub_i = rospy.Publisher('/robofriend/infrared_data', IRSensorData, queue_size = 10) # infrared data
    pub_c = rospy.Publisher('/robofriend/cam_data', CamData, queue_size = 10) # camera data

    odata = Pose()
    odata.x = 0.1
    odata.y = 0.2
    odata.theta = 0.3
    odata.linear_velocity = 2 # up down
    odata.angular_velocity = 0 # left right

    idata = IRSensorData()
    idata.inf_left = 1.20
    idata.inf_middle = 2.30
    idata.inf_right = 3.40

    cdata = CamData()
    cdata.top = 10
    cdata.right = 11
    cdata.bottom = 12
    cdata.left = 13
    cdata.name = "CamName"

    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        # pub_o.publish(odata)
        # pub_i.publish(idata)
        pub_c.publish(cdata)

        srv_resp = None
        rospy.wait_for_service('/robofriend/get_rfid_data')

        try:
            request = rospy.ServiceProxy('/robofriend/get_rfid_data', SrvRFIDData)
            srv_resp = request(True)
        except rospy.ServiceException:
            rospy.logwarn("Service call failed")

        rospy.loginfo("{RobobrainTalkerNode} Service recevied: %s", srv_resp)

        rate.sleep() # make sure the publish rate maintains at the needed frequency

if __name__ == '__main__':
    try:
        Talker()
    except rospy.ROSInterruptException:
        pass
