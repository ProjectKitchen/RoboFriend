from RobobrainNode.RobobrainStateHandler import *
from threading import *
from time import *

class RobobrainFacedetectionDataHandler():

    def __init__(self, robostate, publisher_handler):
        self._top = 0
        self._right = 0
        self._bottom = 0
        self._left = 0
        self._name = None

        self._robostate = robostate
        self._pub = publisher_handler

        self.__start_thread()

        self._facedetection_states = {
            'FACE_SEARCH' : 1, \
            'KNOWN_FACE' : 2, \
            'UNKNOWN_FACE' : 3, \
        }
        self._actual_state = self._facedetection_states["FACE_SEARCH"]

    def process_data(self, data):
        print("[INFO] Class: {} ... Received message: {} {} {} {}".format(self.__class__.__name__, self._top, self._right, self._bottom, self._left, self._name))
        if self._actual_state == self._facedetection_states["FACE_SEARCH"]:
            self._top = data.top
            self._right = data.right
            self._bottom = data.bottom
            self._left = data.left
            self._name = data.name
        else:
            pass


    def __start_thread(self):
        self._thread = Thread(
            target = self.__facedetection_handler_thread
        )
        self._thread.daemon = True
        self._thread.start()

    def __facedetection_handler_thread(self):
        while True:
            while RobobrainStateHandler.robostate == RobobrainStateHandler.robostate["FACEDETECTION"]:
                print("{} - Within Facedetection state\n".format(__class__.__name___)
                if self._actual_state == self._facedetection_states["FACE_SEARCH"]:
                    self.__face_search()

    def __face_search(self):
        pass
        


    #top
    @property
    def top(self):
        return self._top

    @top.setter
    def top(self, value):
        self._top = value

    #right
    @property
    def right(self):
        return self._right

    @right.setter
    def right(self, value):
        self._right = value

    #left
    @property
    def left(arg):
        return self._left

    @left.setter
    def left(self, value):
        self._left = value

    #bottom
    @property
    def bottom(self):
        return self._bottom

    @bottom.setter
    def bottom(self, value):
        self._bottom = value

    #name
    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        self._name = value
