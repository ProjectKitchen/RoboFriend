import rospy
import threading

# import ROS messages
from ros_robofriend.msg import CamData
from ros_robofriend.msg import KeyboardData
from ros_robofriend.msg import BatInfMsgData
from std_msgs.msg import String

# import ROS modules
from RobobrainNode.RobobrainStateHandler import *
from RobobrainNode.RobobrainKeyboardDataHandler import *
from RobobrainNode.RobobrainFacedetectionDataHandler import *
from RobobrainNode.RobobrainBatteryInfraredDataHandler import *
from RobobrainNode.RobobrainPublisherHandler import *

# global variables
robo_state = 0
runFlag = True
topics = {'T_ODOM_DATA': 'T_ODOM_DATA', \
          'T_BAT_INF_DATA': 'T_BAT_INF_DATA', \
          'T_CAM_DATA': 'T_CAM_DATA', \
          'T_KEYB_DATA': 'T_KEYB_DATA', \
          'T_RFID_DATA': 'T_RFID_DATA', \
          'T_SPEECH_DATA' : 'T_SPEECH_DATA', \
          'T_TEENSY_MOTOR_DATA' : 'T_TEENSY_MOTOR_DATA', \
          'T_EARS_LED_DATA' : 'T_EARS_LED_DATA', \
          'T_SERVO_CAM_DATA' : 'T_SERVO_CAM_DATA'}

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

    event = threading.Event()

    publish_handler = RobobrainPublisherHandler(topics)
    robostate = RobobrainStateHandler(event)         # sets actual state to IDLE and starts thread, TODO: include publisher handler as argument!
    keyboard = RobobrainKeyboardDataHandler(event, robostate)
    facedetection = RobobrainFacedetectionDataHandler(robostate, publish_handler)
    battery_infrared = RobobrainBatteryInfraredDataHandler(robostate)
    rospy.Subscriber(topics['T_KEYB_DATA'], KeyboardData, keyboard_data_cb, keyboard)
    rospy.Subscriber(topics['T_CAM_DATA'], CamData, facedetection_data_cb, facedetection)
    rospy.Subscriber(topics['T_BAT_INF_DATA'], BatInfMsgData, battery_infrared_data_cb, battery_infrared)

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
