import threading
import rospy

# import ros message
from robofriend.msg import LedEarsData

# import ros module
from LedEarsNode.LedEarsDataHandler import *

def node_start():
    print("[INFO] ROS Ears/Led Node started!\n")

    # thread to handle incoming messages from brain node
    led_ears_thread = threading.Thread(
        target = handle_ears_led
    )

    # start thread as a daeomon
    led_ears_thread.daemon = True

    # start ears/led thread
    led_ears_thread.start()

def handle_ears_led():

    print("[INFO] Thread for ears_led started !")
    led_ears = LedEarsDataHandler()
    rospy.Subscriber('T_LED_EARS_DATA', LedEarsData, led_ears.process_data)
