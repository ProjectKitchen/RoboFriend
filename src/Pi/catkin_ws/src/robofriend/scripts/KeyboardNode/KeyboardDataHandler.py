import pygame
import rospy
from threading import Lock

# import ROS message
from ros_robofriend.msg import KeyboardData

class KeyboardDataHandler():
    def __init__(self, pub, msg):
        self._pub = pub
        self._msg = msg

    def message_publish(self, message):
        self._msg.quit = message ['quit']
        self._msg.up_down = message ['up_down']
        self._msg.pressed_key = message ['pressed_key']
        self._pub.publish(self._msg)
        print("[INFO] Published Keyboard Message: {}\n".format(self._msg))
