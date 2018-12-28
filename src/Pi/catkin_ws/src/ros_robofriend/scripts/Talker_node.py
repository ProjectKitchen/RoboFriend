#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from turtlesim.msg import Pose
from robofriend.msg import CamData

def Talker():

    rospy.init_node('talker', anonymous = True)
    
    pub_v = rospy.Publisher('T_VOLT_DATA', Float64, queue_size = 10) # battery voltage
    pub_o = rospy.Publisher('T_ODOM_DATA', Pose, queue_size = 10) # odometry data
    pub_i = rospy.Publisher('T_IR_DATA', String, queue_size = 10) # infrared data
    pub_c = rospy.Publisher('T_CAM_DATA', CamData, queue_size = 10) # camera data
    pub_k = rospy.Publisher('T_KEYB_DATA', String, queue_size = 10) # keyboard data
    pub_r = rospy.Publisher('T_RFID_DATA', String, queue_size = 10) # rfid data

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
        rospy.loginfo(hello_str)
        pub_v.publish(hello_str)

        rospy.loginfo(odata)
        pub_o.publish(odata)

        hello_str = "hello infrared"
        rospy.loginfo(hello_str)
        pub_i.publish(hello_str)

        rospy.loginfo(cdata)
        pub_c.publish(cdata)

        hello_str = "hello keyboard"
        rospy.loginfo(hello_str)
        pub_k.publish(hello_str)

        hello_str = "hello rfid"
        rospy.loginfo(hello_str)
        pub_r.publish(hello_str)

        rate.sleep()

if __name__ == '__main__':
    try:
        Talker()
    except rospy.ROSInterruptException:
        pass
