import rospy
import threading

# import ros messages
from ros_robofriend.msg import CamData
from ros_robofriend.msg import KeyboardData
from std_msgs.msg import String

# import modules
import KeyboardDataHandler
#import FacedetectionHandler

# global variables
robo_state = 0
runFlag = True
topics = {'T_VOLT_DATA': 'T_VOLT_DATA', \
          'T_ODOM_DATA': 'T_ODOM_DATA', \
          'T_IR_DATA': 'T_IR_DATA', \
          'T_CAM_DATA': 'T_CAM_DATA', \
          'T_KEYB_DATA': 'T_KEYB_DATA', \
          'T_RFID_DATA': 'T_RFID_DATA' }

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
    args.processData(data)

def facedetection_data_cb(facedetection, data):
    facedetection.processData(data)

def RobobrainHandler():
    global runFlag

    keyboard = KeyboardDataHandler.KeyboardDataHandler()
    #facedetection = FacedetectionHandler.FacedetectionHandler()

    rospy.Subscriber(topics['T_KEYB_DATA'], KeyboardData, keyboard_data_cb, keyboard)
    #rospy.Subscriber(topics['T_CAM_DATA'], CamData, facedetection_data_cb(facedetection))

    while runFlag:
        pass
    #     if keyboard.command != None:
    #         print("[INFO] Keyboard Received data: {} {} {}".format(keyboard.command, keyboard.action, keyboard.action_opt))
    #         keyboard.command = None
