#!/usr/bin/env python

import rospy
import threading

# import ROS messages
# from enum import Enum
from std_msgs.msg import String
from std_msgs.msg import Float64
from robofriend.msg import CamData
from robofriend.msg import KeyboardData
from robofriend.msg import PCBSensorData

# import ROS modules
from RobobrainFacedetectionDataHandler import *
from RobobrainKeyboardDataHandler import *
from RobobrainPCBSensorDataHandler import *
from RobobrainPublisherHandler import *
from RobobrainStateHandler import *

def stopNode():
    rospy.signal_shutdown("Stopping Robobrain node!")

def main():
    rospy.init_node('robofriend_robobrain', anonymous = True)
    rospy.loginfo("Starting Robobrain node!")

    # publish here
    # pub = rospy.Publisher('topic_name', MsgType, queue_size = 10)

    # TODO: do we need a event here?
    robostate  = RobobrainStateHandler()
    bat = RobobrainPCBSensorDataHandler()
    # bat = RobobrainBatteryInfraredDataHandler(robostate)
    # odo = RobobrainOdometryDataHandler.RobobrainOdometryDataHandler()
    # ir  = RobobrainInfraredDataHandler.RobobrainInfraredDataHandler()
    fd = RobobrainFacedetectionDataHandler()
    key = RobobrainKeyboardDataHandler()


    # TODO: this can be managed in an easier way :)
    # publish_handler = RobobrainPublisherHandler(topics)

    rospy.Subscriber("/robofriend/pcb_sensor_data", PCBSensorData, bat.process_data)
    # rospy.Subscriber("/robofriend/odom_data", Pose, odo.processData)
    # rospy.Subscriber("/robofriend/ir_data",   String, ir.processData)
    # rospy.Subscriber("/robofriend/cam_data",  CamData, fd.processData)
    # rospy.Subscriber("/robofriend/keyb_data", KeyboardData, key.processData)
    
    rate = rospy.Rate(0.2) # 200mhz

    while not rospy.is_shutdown():
        # if robostate.state = robostate["IDLE"]
        #
        # if keyboard.command == "move":
        #     publish_handler.teensy_motor_message_publish(keyboard.action, keyboard.action_opt)
        #     keyboard.command = None
        # elif keyboard.command == "speech":
        #     publish_handler.speech_message_publish(keyboard.action, keyboard.action_opt)
        #     keyboard.command = None

        rospy.loginfo("state: {}".format(robostate.state))
        # rospy.loginfo(bat.voltage)
        # pub.publish(bat.voltage)

        rate.sleep() # make sure the publish rate maintains at the needed frequency

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass