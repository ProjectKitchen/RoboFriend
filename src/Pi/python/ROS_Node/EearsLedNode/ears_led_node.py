import threading
import rospy

# import ros message
from ros_robofriend.msg import EarsLedData

# import ros module
from ROS_Node.EearsLedNode.EarsLedDataHandler import *

def node_start():
    print("[INFO] ROS Ears/Led Node started!\n")

    # thread to handle incoming messages from brain node
    ears_led_thread = threading.Thread(
        target = handle_ears_led
    )

    # start thread as a daeomon
    ears_led_thread.daemon = True

    # start ears/led thread
    ears_led_thread.start()

def handle_ears_led():

    print("[INFO] Thread for ears_led started !")
    ears_led = EarsLedDataHandler()
    rospy.Subscriber('T_EARS_LED_DATA', EarsLedData, ears_led.process_data)
