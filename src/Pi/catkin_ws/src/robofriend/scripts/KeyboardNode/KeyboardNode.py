#!/usr/bin/env python3
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


class KeyboardDataHandler():
    def __init__(self):

        self._special_button_key = {
            8 : "backspace", \
            13 : "enter", \
            27 : "escape", \
            32 : "space", \
            273 : "up", \
            274 : "down", \
            275 : "right", \
            276 : "left"
        }

        self._pub = rospy.Publisher('/robofriend/keyb_data', KeyboardData, queue_size = 10)
        self._msg = KeyboardData()

    def _keyboard(self):
        publish_message = {}
        try:
            event = pygame.event.wait()
            if 'mod' in event.__dict__ and 'unicode' in event.__dict__:
                if event.type == pygame.QUIT:
                    rospy.logdebug("{%s} - Quit pressed\n")
                    publish_message = self._message_dict_merge(quit = True)
                if event.__dict__["mod"] == 12288:
                    if event.key in self._special_button_key:
                        publish_message = self._message_dict_merge(up_down = "up", pressed_key = self._special_button_key[event.key])
                    else:
                        if event.__dict__['unicode']:
                            publish_message = self._message_dict_merge(up_down = "up", pressed_key = event.__dict__['unicode'])
                        else:
                            pass
                elif event.type == pygame.KEYDOWN:
                    if event.key in self._special_button_key:
                        publish_message = self._message_dict_merge(pressed_key = self._special_button_key[event.key])
                    else:
                        if event.__dict__['unicode']:
                            publish_message = self._message_dict_merge(pressed_key = event.__dict__['unicode'])
                        else:
                            pass
                else:
                    pass
                if bool(publish_message) == True:
                    self._message_publish(publish_message)
                else:
                    pass
        except Exception as e:
            rospy.logdebug("{%s} - Keyboard exception arised: %s\n",
                self.__class__.__name__, e)

    def _message_dict_merge(self, quit = False, up_down = "down", pressed_key = ""):
        message = {'quit' : quit, \
                   'up_down' : up_down, \
                   'pressed_key' : pressed_key}
        return message

    def _message_publish(self, message):
        self._msg.quit = message ['quit']
        self._msg.up_down = message ['up_down']
        self._msg.pressed_key = message ['pressed_key']
        self._pub.publish(self._msg)
        rospy.logwarn("{%s} - Published Keyboard Message: %s\n",
            self._msg)

def shutdown():
    rospy.loginfo("{%s} - stopping keyboard data handler.", rospy.get_caller_id())
    rospy.signal_shutdown("Stopping Keyboard node!")


def Keyboard():
    rospy.init_node("robofriend_keyboard_node", log_level = rospy.INFO)
    rospy.loginfo("{%s} - starting keyboard node!",
        rospy.get_caller_id())

    kb = KeyboardDataHandler()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        kb._keyboard()

        rate.sleep()

if __name__ == '__main__':
    try:
        Keyboard()
    except rospy.ROSInterruptException:
        pass
