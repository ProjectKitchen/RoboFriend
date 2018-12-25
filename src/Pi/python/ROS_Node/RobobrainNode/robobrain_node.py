import rospy
import threading

# import ROS messages
from ros_robofriend.msg import CamData
from ros_robofriend.msg import KeyboardData
from ros_robofriend.msg import BatInfMsgData
from std_msgs.msg import String

# import ROS modules
from ROS_Node.RobobrainNode.RobobrainStateHandler import *
from ROS_Node.RobobrainNode.RobobrainKeyboardDataHandler import *
from ROS_Node.RobobrainNode.RobobrainFacedetectionDataHandler import *
from ROS_Node.RobobrainNode.RobobrainBatteryInfraredDataHandler import *
from ROS_Node.RobobrainNode.RobobrainPublisherHandler import *

# global variables
robo_state = 0
runFlag = True
topics = {'T_ODOM_DATA': 'T_ODOM_DATA', \
          'T_BAT_INF_DATA': 'T_BAT_INF_DATA', \
          'T_CAM_DATA': 'T_CAM_DATA', \
          'T_KEYB_DATA': 'T_KEYB_DATA', \
          'T_RFID_DATA': 'T_RFID_DATA', \
          'T_SPEECH_DATA' : 'T_SPEECH_DATA', \
          'T_TEENSY_MOTOR_DATA' : 'T_TEENSY_MOTOR_DATA' }

roboState = {
    'IDLE' : 1, \
    'FIND_CHARGING_STATION' : 2, \
    'AUTONOM' : 3, \
    'MANUAL' : 4
}

def node_stop():
    global runFlag

    runFlag = False
    print("[INFO] Stopping robobrain node!")

def node_start():
    global runFlag

    print("[INFO] Robobrain node started!")

    robobrain_thread = threading.Thread(
        target = RobobrainHandler
    )

    # set thread as a daemon
    robobrain_thread.daemon = True

    # start Robobrain thread
    robobrain_thread.start()


def keyboard_data_cb(data, args):
    args.process_data(data)

def facedetection_data_cb(data, args):
    args.process_data(data)

def battery_infrared_data_cb(data, args):
    args.process_data(data)

def RobobrainHandler():
    global runFlag

    publish_handler = RobobrainPublisherHandler(topics)
    robostate = RobobrainStateHandler(roboState['IDLE'])

    keyboard = RobobrainKeyboardDataHandler()
    facedetection = RobobrainFacedetectionDataHandler()
    battery_infrared = RobobrainBatteryInfraredDataHandler()
    rospy.Subscriber(topics['T_KEYB_DATA'], KeyboardData, keyboard_data_cb, keyboard)
    rospy.Subscriber(topics['T_CAM_DATA'], CamData, facedetection_data_cb, facedetection)
    rospy.Subscriber(topics['T_BAT_INF_DATA'], BatInfMsgData, battery_infrared_data_cb, battery_infrared)

    #TODO: Subscriber from Battery/Infrared Node has to be implemneted!!


    while runFlag:
        # if robostate.state = robostate["IDLE"]
        #
        # if keyboard.command == "move":
        #     publish_handler.teensy_motor_message_publish(keyboard.action, keyboard.action_opt)
        #     keyboard.command = None
        # elif keyboard.command == "speech":
        #     publish_handler.speech_message_publish(keyboard.action, keyboard.action_opt)
        #     keyboard.command = None
        pass
