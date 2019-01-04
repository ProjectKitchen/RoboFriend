#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from turtlesim.msg import Pose
from robofriend.msg import CamData

def Talker():
    rospy.init_node('robofriend_talker', anonymous = True)
    rospy.loginfo("Starting Talker node!")

    pub_v = rospy.Publisher('/robofriend/volt_data', Float64, queue_size = 10) # battery voltage
    pub_o = rospy.Publisher('/robofriend/odom_data', Pose, queue_size = 10) # odometry data
    # pub_i = rospy.Publisher('/robofriend/ir_data', String, queue_size = 10) # infrared data
    pub_c = rospy.Publisher('/robofriend/cam_data', CamData, queue_size = 10) # camera data
    # pub_k = rospy.Publisher('/robofriend/keyb_data', String, queue_size = 10) # keyboard data
    # pub_r = rospy.Publisher('/robofriend/rfid_data', String, queue_size = 10) # rfid data

    odata = Pose()
    odata.x = 0.1
    odata.y = 0.2
    odata.theta = 0.3
    odata.linear_velocity = 2 # up down
    odata.angular_velocity = 0 # left right

    cdata = CamData()
    cdata.top = 10
    cdata.right = 11
    cdata.bottom = 12
    cdata.left = 13
    cdata.name = "CamName"

    rate = rospy.Rate(1) # 1hz
    
    while not rospy.is_shutdown():
        hello_str = 3.14
        pub_v.publish(hello_str)

        rospy.loginfo(odata)
        pub_o.publish(odata)

        rospy.loginfo(cdata)
        pub_c.publish(cdata)

        rate.sleep() # make sure the publish rate maintains at the needed frequency

if __name__ == '__main__':
    try:
        Talker()
    except rospy.ROSInterruptException:
        pass
