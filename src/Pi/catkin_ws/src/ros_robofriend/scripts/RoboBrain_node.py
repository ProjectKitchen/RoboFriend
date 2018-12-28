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
	
    # publish here
	# pub = rospy.Publisher('topic_name', MsgType, queue_size = 10)
	
    rb  = RoboBrain(RoboStates.INIT)
    bat = BatteryVoltageDataHandler.BatteryVoltageDataHandler()
    odo = OdometryDataHandler.OdometryDataHandler()
    ir  = InfraredDataHandler.InfraredDataHandler()
    cam = CameraDataHandler.CameraDataHandler()
    key = KeyboardDataHandler.KeyboardDataHandler()
    rf  = RFIDDataHandler.RFIDDataHandler()

    rospy.Subscriber("/robofriend/volt_data", Float64, bat.processData)
    rospy.Subscriber("/robofriend/odom_data", Pose, odo.processData)
    rospy.Subscriber("/robofriend/ir_data",   String, ir.processData)
    rospy.Subscriber("/robofriend/cam_data",  CamData, cam.processData)
    rospy.Subscriber("/robofriend/keyb_data", String, key.processData)
    rospy.Subscriber("/robofriend/rfid_data", String, rf.processData)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

    rate = rospy.Rate(0.2) # 200mhz
    while not rospy.is_shutdown():
        rospy.loginfo("state: {}".format(rb.state))
        rospy.loginfo(bat.voltage)
        # pub.publish(bat.voltage)

        rate.sleep() # make sure the publish rate maintains at the needed frequency

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass