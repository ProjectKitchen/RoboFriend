from RobobrainStateHandler import *
import rospy

class RobobrainKeyboardDataHandler():

    def __init__(self, sh, event, queue):
        self._idle_event = event
        self._statehandler = sh
        self._queue = queue

    def process_data(self, data):
        rospy.logdebug("{%s} - Keyboard messages received: %s", self.__class__.__name__, data)
        self._input_handler(data)

    def _input_handler(self, data):
        if self._statehandler.state == RobobrainStateHandler.robostate["IDLE"]:
            self._idle_event.set()      # set event to stay in IDLE State

            self._process_input_idle(data)

            #TODO: Input processing according to the IDLE State

        elif self._statehandler.state == RobobrainStateHandler.robostate['FACEDETECTION']:
            self._queue.put(data.pressed_key)
