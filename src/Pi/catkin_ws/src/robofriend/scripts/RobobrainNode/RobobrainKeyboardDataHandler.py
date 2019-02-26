# from RobobrainStateHandler import *
from RobobrainNode.RobobrainStateHandler import *


class RobobrainKeyboardDataHandler():

    def __init__(self, sh, event, queue):
        self.__quit = None
        self.__up_down = None
        self.__pressed_key = None

        self.__event = event
        self._statehandler = sh
        self.__queue = queue

    def process_data(self, data):
        self.__quit = data.quit
        self.__up_down = data.up_down
        self.__pressed_key = data.pressed_key
        print("[INFO] Class: {} ... Received message from keyboard-node: {}\n".format(self.__class__.__name__, data))
        self.__input_handler(data)

    def __input_handler(self, data):
        #print("[INFO] Within keyboard input handler!")
        #print("[INFO] actual State: {}".format(self._statehandler.state))
        if self._statehandler.state == RobobrainStateHandler.robostate["IDLE"]:
            self.__event.set()
            print("[INFO] Event set!")
            #TODO: Input processing according to the IDLE State
        elif self._statehandler.state == RobobrainStateHandler.robostate['FACEDETECTION']:
            self.__queue.put(self.__pressed_key)
