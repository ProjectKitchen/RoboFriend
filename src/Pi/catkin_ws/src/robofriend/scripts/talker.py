#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from robofriend.msg import CamData
from robofriend.msg import OdoData

def talker():
    # battery voltage
    pub_v = rospy.Publisher('T_VOLT_DATA', Float64, queue_size = 10)
    # odometry data
    pub_o = rospy.Publisher('T_ODOM_DATA', OdoData, queue_size = 10)
    # infrared data
    pub_i = rospy.Publisher('T_IR_DATA', String, queue_size = 10)
    # camera data
    pub_c = rospy.Publisher('T_CAM_DATA', CamData, queue_size = 10)
    # keyboard data
    pub_k = rospy.Publisher('T_KEYB_DATA', String, queue_size = 10)
    # rfid data
    pub_r = rospy.Publisher('T_RFID_DATA', String, queue_size = 10)

    rospy.init_node('talker', anonymous = True)
    rate = rospy.Rate(1) # 1hz

    odata = OdoData()
    odata.x = 0.1
    odata.y = 0.2
    odata.z = 0.3

    cdata = CamData()
    cdata.top = 10
    cdata.right = 11
    cdata.bottom = 12
    cdata.left = 13
    cdata.name = "CamName"
    
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
        talker()
    except rospy.ROSInterruptException:
        pass
