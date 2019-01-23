# external modules
import pygame
import sys
import threading
import re
import traceback
import queue
import rospy

# import ros message
from robofriend.msg import KeyboardData

# own modules
from KeyboardNode.KeyboardDataHandler import *

# globals
runFlag = True

def node_stop():
    global runFlag
    print("[INFO] Stopping keboard node!")
    runFlag = False

def node_start():
    print("[INFO] ROS Keyboard Node started!\n")

    # thread to handle the keyboard inputs
    keyboard_thread = threading.Thread(
        target = handle_keyboard
    )

    #start thread as daemon
    keyboard_thread.daemon = True

    # start keyboard thread
    keyboard_thread.start()

def handle_keyboard():
    global runFlag

    # special buttons which are processed others ignored
    special_button_key = {
        8 : "backspace", \
        13 : "enter", \
        27 : "escape", \
        32 : "space", \
        273 : "up", \
        274 : "down", \
        275 : "right", \
        276 : "left"
    }

    publish_message = {}

    pub = rospy.Publisher('T_KEYB_DATA', KeyboardData, queue_size = 10)
    msg = KeyboardData()

    keyboard = KeyboardDataHandler(pub, msg)

    while runFlag:
        try:
            event = pygame.event.wait()
            if 'mod' in event.__dict__ and 'unicode' in event.__dict__:
                if event.type == pygame.QUIT:
                    print("[INFO] Quit pressed")
                    publish_message = message_dict_merge(quit = True)
                if event.__dict__["mod"] == 12288:
                    if event.key in special_button_key:
                        publish_message = message_dict_merge(up_down = "up", pressed_key = special_button_key[event.key])
                    else:
                        if event.__dict__['unicode']:
                            publish_message = message_dict_merge(up_down = "up", pressed_key = event.__dict__['unicode'])
                        else:
                            pass
                elif event.type == pygame.KEYDOWN:
                    if event.key in special_button_key:
                        publish_message = message_dict_merge(pressed_key = special_button_key[event.key])
                    else:
                        if event.__dict__['unicode']:
                            publish_message = message_dict_merge(pressed_key = event.__dict__['unicode'])
                        else:
                            pass
                else:
                    pass
                if bool(publish_message) == True:
                    keyboard.message_publish(publish_message)
                    publish_message.clear()
                else:
                    pass
        except Exception as e:
            print("keyboard exception: {}".format(e))
            print(traceback.format_exc())

def message_dict_merge(quit = False, up_down = "down", pressed_key = ""):
    message = {'quit' : quit, \
               'up_down' : up_down, \
               'pressed_key' : pressed_key}
    return message
