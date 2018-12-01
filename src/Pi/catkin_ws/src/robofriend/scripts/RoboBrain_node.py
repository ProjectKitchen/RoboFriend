#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64
from robofriend.msg import CamData
from turtlesim.msg import Pose

import RoboStates
import BatteryVoltageDataHandler
import OdometryDataHandler
import InfraredDataHandler
import CameraDataHandler
import KeyboardDataHandler
import RFIDDataHandler

robo_state = 0

topics = {'T_VOLT_DATA': 'T_VOLT_DATA', \
          'T_ODOM_DATA': 'T_ODOM_DATA', \
          'T_IR_DATA': 'T_IR_DATA', \
          'T_CAM_DATA': 'T_CAM_DATA', \
          'T_KEYB_DATA': 'T_KEYB_DATA', \
          'T_RFID_DATA': 'T_RFID_DATA' }

def set_state(state):
    robo_state = state

def voltage_data_cb(data):
    bat.processData(data)
    
def odometry_data_cb(data):
    odo.processData(data)
    
def infrared_data_cb(data):
    ir.processData(data)
    
def camera_data_cb(data):
    cam.processData(data)
    
def keyboard_data_cb(data):
    key.processData(data)
    
def rfid_data_cb(data):
    rf.processData(data)

def listener():
    # test 1 voltage
    pub_1 = rospy.Publisher('T_TEST_1_DATA', Float64, queue_size = 10)
    # test 2 data
    pub_2 = rospy.Publisher('T_TEST_2_DATA', String, queue_size = 10)
    # pathplanner data
    pub_p = rospy.Publisher('T_PATH_DATA', String, queue_size = 10)

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'RoboBrain' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('robobrain', anonymous = True)

    rospy.Subscriber(topics['T_VOLT_DATA'], Float64, voltage_data_cb)
    rospy.Subscriber(topics['T_ODOM_DATA'], Pose, odometry_data_cb)
    rospy.Subscriber(topics['T_IR_DATA'],   String, infrared_data_cb)
    rospy.Subscriber(topics['T_CAM_DATA'],  CamData, camera_data_cb)
    rospy.Subscriber(topics['T_KEYB_DATA'], String, keyboard_data_cb)
    rospy.Subscriber(topics['T_RFID_DATA'], String, rfid_data_cb)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

    rate = rospy.Rate(0.2) # 200mhz
    while not rospy.is_shutdown():
        rospy.loginfo("========================")

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
    bat = BatteryVoltageDataHandler.BatteryVoltageDataHandler()
    odo = OdometryDataHandler.OdometryDataHandler()
    ir  = InfraredDataHandler.InfraredDataHandler()
    cam = CameraDataHandler.CameraDataHandler()
    key = KeyboardDataHandler.KeyboardDataHandler()
    rf  = RFIDDataHandler.RFIDDataHandler()

    try:
        listener()
    except rospy.ROSInterruptException:
        pass