# external modules
import pygame
import sys
import threading
import re
import traceback
import queue
import rospy

# own modules
import teensyCommunicator
import faceModule
import soundModule
import speechModule
import systemModule

# import ros message
from ros_robofriend.msg import KeyboardData

# globals
runFlag = True

class Keyboard_Publisher():
    def __init__(self, pub, msg):
        self._pub = pub
        self._msg = msg

    def message_publish(self, message):
        self._msg.command = message ['command']
        self._msg.action = message ['action']
        self._msg.action_opt = message ['action_opt']
        self._pub.publish(self._msg)
        print("[INFO] Published Keyboard Message: {}".format(self._msg))

def node_stop():
    global runFlag
    print("[INFO] Stopping keboard node!")
    runFlag = False

def node_start():
    print("[INFO] ROS Keyboard Node started!\n")

    pub = rospy.Publisher('T_KEYB_DATA', KeyboardData, queue_size = 10)

    msg = KeyboardData()

    keyboard = Keyboard_Publisher(pub, msg)

    # thread to handle the keyboard inputs
    keyboard_thread = threading.Thread(
        target = handle_keyboard,
        args = (keyboard, )
    )

    #start thread as daemon
    keyboard_thread.daemon = True

    # start keyboard thread
    keyboard_thread.start()

def handle_keyboard(keyboard):
    global runFlag

    speechBuffer = ""
    shutdownKeyword = "exit"        # system shutdown
    quitKeyword = "quit"            # process shutdown
    lastSay = None
    publish_message = {}

    while runFlag:
        try:
            event = pygame.event.wait()
            if event.type == pygame.QUIT:
                publish_message = message_dict_merge(quitKeyword)
            if event.type == pygame.KEYUP and event.key in [pygame.K_DOWN, pygame.K_LEFT, pygame.K_RIGHT, pygame.K_UP]:
                print('move stop via keyboard')
                publish_message = message_dict_merge('move', 'stop')
            if event.type == pygame.KEYDOWN:
                print('***** Key press recognized: ')

                if event.key == pygame.K_RETURN:            # enter buton
                    if speechBuffer == shutdownKeyword:     # system shutdown
                        print("[INFO] Within Shutdwodn Keyword!!")
                        publish_message = message_dict_merge(shutdownKeyword)
                    elif speechBuffer == quitKeyword:       # process shutdown
                        publish_message = message_dict_merge(quitKeyword)
                    elif speechBuffer:                      # when speechbuffer is not empty
                        publish_message = message_dict_merge('speech', 'custom', speechBuffer)
                        lastSay = speechBuffer
                    speechBuffer = ''
                elif event.key == pygame.K_ESCAPE:          # esc button
                    print('clearing speech buffer...')
                    speechBuffer = ''
                elif event.key == pygame.K_BACKSPACE:
                    if lastSay:
                        publish_message = message_dict_merge('speech', 'custom', lastSay)

                # ------------ move ---------------
                elif event.key == pygame.K_DOWN:
                    print('move back via keyboard')
                    publish_message = message_dict_merge('move', 'backword', 'loop')
                elif event.key == pygame.K_LEFT:
                    print("move left via keyboard")
                    publish_message = message_dict_merge('move', 'left', 'loop')
                elif event.key == pygame.K_RIGHT:
                    print('move right via keyboard')
                    publish_message = message_dict_merge('move', 'right', 'loop')
                elif event.key == pygame.K_UP:
                    print("move forward via keyboard")
                    publish_message = message_dict_merge('move', 'forward', 'loop')

                # ------------ face ---------------
                elif event.unicode == ',':
                    print('smile increase via keyboard')
                    publish_message = message_dict_merge('face', 'smile', 'increase')
                elif event.unicode == '.':
                    print('smile decrease via keyboard')
                    publish_message = message_dict_merge('face', 'smile', 'decrease')

                # ------------ sounds ---------------
                elif event.unicode == '-':
                    publish_message = message_dict_merge('sound', 'random')
                elif event.unicode == '#':
                    publish_message = message_dict_merge('sound', 'last')

                # ------------ speech ---------------
                elif event.unicode == '1':
                    publish_message = message_dict_merge('speech', 'custom', 'Hallo wie gehts?')
                elif event.unicode == '2':
                    publish_message = message_dict_merge('speech', 'custom', 'Danke, mir geht es gut')
                elif event.unicode == '3':
                    publish_message = message_dict_merge('speech', 'custom', 'Willst du etwas zum knabbern?')
                elif event.unicode == '4':
                    publish_message = message_dict_merge('speech', 'custom', 'Bitte, gerne')
                elif event.unicode == '5':
                    publish_message = message_dict_merge('speech', 'custom', 'Wie heisst du?')
                elif event.unicode == '6':
                    publish_message = message_dict_merge('speech', 'custom', 'Ich heisse Robofreund')
                elif event.unicode == '7':
                    publish_message = message_dict_merge('speech', 'custom', 'Mir ist langweilig')
                elif event.unicode == '8':
                    publish_message = message_dict_merge('speech', 'custom', 'Heute ist ein schoener Tag')
                elif event.unicode == '9':
                    publish_message = message_dict_merge('speech', 'random')
                elif event.unicode == '0':
                    publish_message = message_dict_merge('speech', 'bullshit')
                elif re.match('^[a-zA-Z ]$', event.unicode):
                    speechBuffer += event.unicode                       # concatenate letter
                    print("[INFO] Speech buffer is now: {}".format(speechBuffer))

            if bool(publish_message) == True:       # cheks if dictionary is empty or not
                keyboard.message_publish(publish_message)
                publish_message.clear()
            else:
                pass
        except Exception as e:
            print("keyboard exception: {}".format(e))
            print(traceback.format_exc())

def message_dict_merge(command, action = "", action_opt = ""):
    message = {'command' : command, \
               'action' : action, \
               'action_opt' : action_opt}
    return message
