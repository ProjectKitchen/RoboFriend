#!/usr/bin/env python
import rospy

# import ROS messages
from sensor_msgs.msg import BatteryState
from robofriend.msg import IRSensorData
from robofriend.msg import CamData
# from robofriend.msg import KeyboardData

# import ROS modules
from RobobrainFacedetectionDataHandler import *
from RobobrainKeyboardDataHandler import *
from RobobrainPCBSensorDataHandler import *
from RobobrainPublisherHandler import *
from RobobrainStateHandler import *

# TODO: import constants

class RoboBrain(object):
    def __init__(self, state):
        self._state = state

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, value):
        self._state = value

def shutdown():
    rospy.signal_shutdown("Stopping Robobrain node!")

def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'RoboBrain' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('robofriend_robobrain', anonymous = True, log_level = rospy.INFO)
    rospy.loginfo("Starting Robobrain node!")

    # publish here
    # pub = rospy.Publisher('topic_name', MsgType, queue_size = 10)

    # TODO: do we need a event here?
    # robostate  = RobobrainStateHandler()
    bat = RobobrainPCBSensorDataHandler()
    # odo = RobobrainOdometryDataHandler()
    # ir  = RobobrainInfraredDataHandler()
    fd = RobobrainFacedetectionDataHandler()
    key = RobobrainKeyboardDataHandler()


    # TODO: this can be managed in an easier way :)
    # publish_handler = RobobrainPublisherHandler(topics)

    rospy.Subscriber("/robofriend/battery_state", BatteryState, bat.process_bs_data)
    rospy.Subscriber("/robofriend/infrared_data", IRSensorData, bat.process_ir_data)
    # rospy.Subscriber("/robofriend/odom_data", Pose, odo.process_data)
    # rospy.Subscriber("/robofriend/ir_data",   String, ir.process_data)
    rospy.Subscriber("/robofriend/cam_data",  CamData, fd.process_data)
    # rospy.Subscriber("/robofriend/keyb_data", KeyboardData, key.process_data)
    
    rate = rospy.Rate(0.2) # 200mhz

    while not rospy.is_shutdown():
        if bat.power_supply_status == 5:
            rospy.loginfo("{RobobrainNode} Battery overcharged")
        elif bat.power_supply_status == 4:
            rospy.loginfo("{RobobrainNode} Battery full")
        elif bat.power_supply_status == 3:
            rospy.loginfo("{RobobrainNode} Battery good")
        elif bat.power_supply_status == 2:
            rospy.loginfo("{RobobrainNode} Battery warning")
        elif bat.power_supply_status == 1:
            rospy.loginfo("{RobobrainNode} Battery critical")
        elif bat.power_supply_status == 0:
            rospy.loginfo("{RobobrainNode} Battery Unknown")

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