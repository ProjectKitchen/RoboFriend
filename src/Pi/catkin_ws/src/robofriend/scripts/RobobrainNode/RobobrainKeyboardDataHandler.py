from RobobrainNode.RobobrainStateHandler import *


class RobobrainKeyboardDataHandler():

    def __init__(self, event, robostate, queue):
        self.__quit = None
        self.__up_down = None
        self.__pressed_key = None

        self.__event = event
        self.__robostate_obj = robostate
        self.__queue = queue

    def process_data(self, data):
        self.__quit = data.quit
        self.__up_down = data.up_down
        self.__pressed_key = data.pressed_key
        print("[INFO] Class: {} ... Received message from keyboard-node: {}\n".format(self.__class__.__name__, data))
        self.__input_handler(data)

    def __input_handler(self, data):
        if self.__robostate_obj.state == RobobrainStateHandler.robostate["IDLE"]:
            self.__event.set()
            print("[INFO] Event set!")
            #TODO: Input processing according to the IDLE State
        elif self.__robostate_obj.state == RobobrainStateHandler.robostate['FACEDETECTION']:
            self.__queue.put(self.__pressed_key)

    # command
    @property
    def quit(self):
        return self.__quit

    @quit.setter
    def quit(self, value):
        self.__quit = value

    # up_down
    @property
    def up_down(self):
        return self.__up_down

    @up_down.setter
    def up_down(self, value):
        self._action = value

    #action_opt
    @property
    def pressed_key(self):
        return self.__pressed_key

    @pressed_key.setter
    def pressed_key(self, value):
        self.__pressed_key = value
