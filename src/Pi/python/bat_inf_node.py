import rospy
import threading
import time
import BatteryInfraredDataHandler

#import ros service
from ros_robofriend.srv import BatInfData

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

    #TODO: Publisher for Central node is needed

    bat_inf = BatteryInfraredDataHandler.BatteryInfraredDataHandler()

    while runFlag:
        bat_inf.request_sensor_values()
        time.sleep(5)
