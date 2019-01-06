#!/usr/bin/env python

import rospy
import threading

# import ROS messages
# from enum import Enum
from std_msgs.msg import String
from std_msgs.msg import Float64
from sensor_msgs.msg import BatteryState
from robofriend.msg import IRSensorData
from robofriend.msg import CamData
from robofriend.msg import KeyboardData

# import ROS modules
from RobobrainFacedetectionDataHandler import *
from RobobrainKeyboardDataHandler import *
from RobobrainPCBSensorDataHandler import *
from RobobrainPublisherHandler import *
from RobobrainStateHandler import *

# TODO: import constants

def shutdown():
    rospy.signal_shutdown("Stopping Robobrain node!")

def main():
    rospy.init_node('robofriend_robobrain', anonymous = True, log_level = rospy.INFO)
    rospy.loginfo("Starting Robobrain node!")

    # publish here
    # pub = rospy.Publisher('topic_name', MsgType, queue_size = 10)

    # TODO: do we need a event here?
    robostate  = RobobrainStateHandler()
    bat = RobobrainPCBSensorDataHandler()
    # odo = RobobrainOdometryDataHandler()
    # ir  = RobobrainInfraredDataHandler()
    fd = RobobrainFacedetectionDataHandler()
    key = RobobrainKeyboardDataHandler()


    # TODO: this can be managed in an easier way :)
    # publish_handler = RobobrainPublisherHandler(topics)

    rospy.Subscriber("/robofriend/battery_state", BatteryState, bat.process_bs_data)
    rospy.Subscriber("/robofriend/infrared_data", IRSensorData, bat.process_ir_data)
    # rospy.Subscriber("/robofriend/odom_data", Pose, odo.processData)
    # rospy.Subscriber("/robofriend/ir_data",   String, ir.processData)
    # rospy.Subscriber("/robofriend/cam_data",  CamData, fd.processData)
    # rospy.Subscriber("/robofriend/keyb_data", KeyboardData, key.processData)
    
    rate = rospy.Rate(0.2) # 200mhz

    while not rospy.is_shutdown():
        if bat.power_supply_status == 5:
            rospy.loginfo("Battery overcharged")
        elif bat.power_supply_status == 4:
            rospy.loginfo("Battery full")
        elif bat.power_supply_status == 3:
            rospy.loginfo("Battery good")
        elif bat.power_supply_status == 2:
            rospy.loginfo("Battery warning")
        elif bat.power_supply_status == 1:
            rospy.loginfo("Battery critical")
        elif bat.power_supply_status == 0:
            rospy.loginfo("Battery Unknown")

        # if robostate.state = robostate["IDLE"]
        #
        # if keyboard.command == "move":
        #     publish_handler.teensy_motor_message_publish(keyboard.action, keyboard.action_opt)
        #     keyboard.command = None
        # elif keyboard.command == "speech":
        #     publish_handler.speech_message_publish(keyboard.action, keyboard.action_opt)
        #     keyboard.command = None

        rate.sleep() # make sure the publish rate maintains at the needed frequency

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass