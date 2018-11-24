# external modules
import pygame
import sys
import threading
import re
import traceback
import queue
import rospy
from std_msgs import String

# own modules
import teensyCommunicator
import faceModule
import soundModule
import speechModule
import systemModule

# globals
runFlag = True

def node_start():
    print("[INFO] ROS Keyboard Node started!\n")

    pub = rospy.Publisher('keyboard_topic', String, queue_size = 10)
    rospy.init('Keyboard_node', anonymous = True)

    # queue to ensure a communication between keyboard thread and publisher
    thread_queue = queue.Queue()

    # thread which handles the keyboard inputs
    keyboard_thread = threading.Thread(
        target = handle_keyboard,
        args = (thread_queue, )
    )

    # start the keyboard threaded
    keyboard_thread.start()

    while runFlag:
        received_message = thread_queue.get()
        print("[INFO] Received message: {}".format(received_message))
        pub.publisher(received_message)
