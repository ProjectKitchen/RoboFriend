import rospy
import threading
import time

#import ros service
from ros_robofriend.srv import BatInfData

# import ros modules
from BatteryInfraredNode.BatteryInfraredDataHandler import *

# global variables
runFlag = True

def node_stop():
    global runFlag
    print("[INFO] Stopping battery/infrared node!")
    runFlag = False

def node_start():
    print("[INFO] ROS Battery/Infrared Node started!\n")

    # thread to handle the incoming mesages from teensy
    batinf_thread = threading.Thread(
        target = handle_batinf
    )

    # start thread as a daemon
    batinf_thread.daemon = True

    # start battery/infrared thread
    batinf_thread.start()

def handle_batinf():

    bat_inf = BatteryInfraredDataHandler()

    while runFlag:
        bat_inf.request_sensor_values()
        time.sleep(1)
