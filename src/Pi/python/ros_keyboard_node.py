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

    # queue to ensure a communication system between keyboard thread and publisher
    thread_queue = queue.Queue()

    # thread to handle the keyboard inputs
    keyboard_thread = threading.Thread(
        target = handle_keyboard,
        args = (thread_queue, )
    )

    # start keyboard thread
    keyboard_thread.start()

    while runFlag:
        received_message = thread_queue.get()
        print("[INFO] Transmitted data from keyboard node: {}".format(received_message))
        pub.publisher(received_message)

def handle_keyboard(queue):
    global runFlag

    speechBuffer = ""
    shutdownKeyword = "exit"
    quitKeyword = "quit"
    lastSay = None
    command = ""
    tmp_command = ""
    speech_own_message = "speech;own;"

    while runFlag:
        try:
            event = pygame.event.wait()
            if event.type == pygame.QUIT:
                command = message_merge("shutdown")             # Message 1
                #pygame.quit()
                #sys.exit()
            if event.type == pygame.KEYUP and event.key in [pygame.K_DOWN, pygame.K_LEFT, pygame.K_RIGHT, pygame.K_UP]:
                print('move stop via keyboard')
                comamnd = message_merge("move;stop")             # Message 2
                #teensyCommunicator.stopMovement()
            if event.type == pygame.KEYDOWN:
                print('***** Key press recognized: ')

                if event.key == pygame.K_RETURN:
                    if speechBuffer == shutdownKeyword:
                        command = message_merge("shutdown")     # same as Message 1
                        #systemModule.shutdown()
                    if speechBuffer == quitKeyword:
                        command = message_merge("quit")         # Message 4
                        pygame.quit()
                        sys.exit()
                    elif speechBuffer:
                        tmp_command = "speech;own;" + speechBuffer
                        command = message_merge(tmp_command)        # Message 5
                        #speechModule.speak(speechBuffer)
                        lastSay = speechBuffer
                    speechBuffer = ''
                elif event.key == pygame.K_ESCAPE:
                    print('clearing speech buffer...')
                    speechBuffer = ''
                elif event.key == pygame.K_BACKSPACE:
                    if lastSay:
                        tmp_command = "sppech;own;" + lastSay
                        command = message_merge(tmp_command)
                        #speechModule.speak(lastSay)                # same as Message 5

                # ------------ move ---------------
                elif event.key == pygame.K_DOWN:
                    print('move back via keyboard')
                    command = message_merge("move;backword;loop")    # Message 6
                    #teensyCommunicator.moveBackLoop()
                elif event.key == pygame.K_LEFT:
                    print("move left via keyboard")
                    command = message_merge("move;left;loop")        # Message 7
                    #teensyCommunicator.moveLeftLoop()
                elif event.key == pygame.K_RIGHT:
                    print('move right via keyboard
                    command = message_merge("move;right;loop")       # Message 8
                    #teensyCommunicator.moveRightLoop()
                elif event.key == pygame.K_UP:
                    print("move forward via keyboard")
                    command = message_merge("move;forward;loop")     # Message 9
                    #teensyCommunicator.moveForwardLoop()

                # ------------ face ---------------
                elif event.unicode == ',':
                    print('smile increase via keyboard')
                    command = message_merge("face;smile;increase")   # Message 10
                    #faceModule.increaseSmile()
                elif event.unicode == '.':
                    print('smile decrease via keyboard')
                    command = message_merge("face;smile;increase")   # Message 11
                    #faceModule.decreaseSmile()

                # ------------ sounds ---------------
                elif event.unicode == '-':
                    command = message_merge("sound;random")         # Message 12
                    #soundModule.playRandom()
                elif event.unicode == '#':
                    command = message_merge("sound;last")           # Message 13
                    #soundModule.playLastRandom()

                # ------------ speech ---------------
                elif event.unicode == '1':
                    tmp_command = speech_own_message + "Hallo, wie gehts?"
                    command = message_merge(tmp_command)            # Message 14
                    #speechModule.speak('Hallo, wie gehts?')
                elif event.unicode == '2':
                    tmp_command = speech_own_message + "Danke, mir geht es gut"
                    command = message_merge(tmp_command)            # Message 15
                    #speechModule.speak('Danke, mir geht es gut.')
                elif event.unicode == '3':
                    tmp_command = speech_own_message + "Willst du etwas zum knabbern?"
                    command = message_merge(tmp_command)           # Message 16
                    #speechModule.speak('Willst du etwas zum knabbern?')
                elif event.unicode == '4':
                    tmp_command = speech_own_message + "Bitte, gerne"
                    command = message_merge(tmp_command)          # Message 17
                    #speechModule.speak('Bitte, gerne.')
                elif event.unicode == '5':
                    tmp_command = speech_own_message + "Wie heisst du?"
                    command = message_merge(tmp_command)          # Message 18
                    #speechModule.speak('Wie heisst du?')
                elif event.unicode == '6':
                    tmp_command = speech_own_message + "Ich heisse Robofreund"
                    command = message_merge(tmp_command)          # Message 19
                    #speechModule.speak('Ich heisse Robofreund.')
                elif event.unicode == '7':
                    tmp_command = speech_own_message + "Mir ist langweilig"
                    command = message_merge(tmp_command)          # Message 20
                    #speechModule.speak('Mir ist langweilig')
                elif event.unicode == '8':
                    tmp_command = speech_own_message + "Heute ist ein schoner Tag"
                    command = message_merge(tmp_command)          # Message 21
                    #speechModule.speak('Heute ist ein schoener Tag.')
                elif event.unicode == '9':
                    command = message_merge("speech;random")     # Message 22
                    #speechModule.speakRandom()
                elif event.unicode == '0':
                    command = message_merge("speech;bullshit")   # Message 23
                    #speechModule.speakBullshit()
                elif re.match('^[a-zA-Z ]$', event.unicode):
                    speechBuffer += event.unicode
                    print('speech buffer is now: ' + str(speechBuffer))
            queue_transmit(queue, send_message)
            command = ""
            tmp_command = ""
        except Exception as e:
            print('keyboard exception: ' + str(e))
            print(traceback.format_exc())

def node_stop():
    global runFlag
    print("[INFO] stopping keboard node!")
    runFlag = False

def message_merge(command):
    return "keyboard;" + command

def queue_transmit(queue, message):
    queue.put(message)
