#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from turtlesim.msg import Pose
from robofriend.msg import CamData

from enum import Enum
# import RoboStates
import BatteryVoltageDataHandler
import OdometryDataHandler
import InfraredDataHandler
import CameraDataHandler
import KeyboardDataHandler
import RFIDDataHandler

class RoboStates(Enum):
    INIT = 0
    ADMIN = 1
    FIND_CHARGING_STATION = 2
    AUTONOM = 3
    MANUAL = 4

class RoboBrain(object):
    def __init__(self, state):
        self._state = state

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        self._state = value

def main():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'RoboBrain' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('robobrain', anonymous = True)
    
    pub_1 = rospy.Publisher('T_TEST_1_DATA', Float64, queue_size = 10) # test 1 voltage
    pub_2 = rospy.Publisher('T_TEST_2_DATA', String, queue_size = 10) # test 2 data
    pub_p = rospy.Publisher('T_PATH_DATA', String, queue_size = 10) # pathplanner data

    topics = {'T_VOLT_DATA': 'T_VOLT_DATA', \
              'T_ODOM_DATA': 'T_ODOM_DATA', \
              'T_IR_DATA': 'T_IR_DATA', \
              'T_CAM_DATA': 'T_CAM_DATA', \
              'T_KEYB_DATA': 'T_KEYB_DATA', \
              'T_RFID_DATA': 'T_RFID_DATA' }
    
    rb  = RoboBrain(RoboStates.INIT)
    bat = BatteryVoltageDataHandler.BatteryVoltageDataHandler()
    odo = OdometryDataHandler.OdometryDataHandler()
    ir  = InfraredDataHandler.InfraredDataHandler()
    cam = CameraDataHandler.CameraDataHandler()
    key = KeyboardDataHandler.KeyboardDataHandler()
    rf  = RFIDDataHandler.RFIDDataHandler()

    rospy.Subscriber(topics['T_VOLT_DATA'], Float64, bat.processData)
    rospy.Subscriber(topics['T_ODOM_DATA'], Pose, odo.processData)
    rospy.Subscriber(topics['T_IR_DATA'],   String, ir.processData)
    rospy.Subscriber(topics['T_CAM_DATA'],  CamData, cam.processData)
    rospy.Subscriber(topics['T_KEYB_DATA'], String, key.processData)
    rospy.Subscriber(topics['T_RFID_DATA'], String, rf.processData)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

    rate = rospy.Rate(0.2) # 200mhz
    while not rospy.is_shutdown():
        rospy.loginfo("======================== state: {}".format(rb.state))

        rospy.loginfo(bat.voltage)
        pub_1.publish(bat.voltage)

        hello_str = "hello test 2"
        rospy.loginfo(hello_str)
        pub_2.publish(hello_str)

        hello_str = "hello path planner"
        rospy.loginfo(hello_str)
        pub_p.publish(hello_str)

        rospy.loginfo("========================")

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass