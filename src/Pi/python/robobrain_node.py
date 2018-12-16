import rospy
import threading

# import ros messages
from ros_robofriend.msg import CamData
from ros_robofriend.msg import KeyboardData
from std_msgs.msg import String

# import modules
import RobobrainPublisherHandler
import KeyboardDataHandler
import FacedetectionDataHandler

# global variables
robo_state = 0
runFlag = True
topics = {'T_VOLT_DATA': 'T_VOLT_DATA', \
          'T_ODOM_DATA': 'T_ODOM_DATA', \
          'T_IR_DATA': 'T_IR_DATA', \
          'T_CAM_DATA': 'T_CAM_DATA', \
          'T_KEYB_DATA': 'T_KEYB_DATA', \
          'T_RFID_DATA': 'T_RFID_DATA', \
          'T_SPEECH_DATA' : 'T_SPEECH_DATA' }       ### Actuators ###

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

def RobobrainHandler():
    global runFlag

    keyboard = KeyboardDataHandler.KeyboardDataHandler()
    facedetection = FacedetectionDataHandler.FacedetectionDataHandler()
    rospy.Subscriber(topics['T_KEYB_DATA'], KeyboardData, keyboard_data_cb, keyboard)
    rospy.Subscriber(topics['T_CAM_DATA'], CamData, facedetection_data_cb, facedetection)
    #TODO: Subscriber from Battery/Infrared Node has to be implemneted!!

    publish_handler = RobobrainPublisherHandler.RobobrainPublisherHandler()

    while runFlag:
        if keyboard.command == "move":
            publish_handler.teensy_motor_message_publish(keyboard.action, keyboard.action_opt)
            keyboard.command = None
        elif keyboard.command == "speech":
            publish_handler.speech_message_publish(keyboard.action, keyboard.action_opt)
            keyboard.command = None
