from RobobrainStateHandler import *
import rospy

class RobobrainVoicedetectionHandler():

    def __init__(self, sh, queue):
        self._statehandler = sh
        self._queue = queue

    def process_data(self, data):
        rospy.logdebug("{%s} - Message received from Voice Detection Node: {%s}",
            self.__class__.__name__, data)
        self._input_handler(data)

    def _input_handler(self, data):
        if self._statehandler.state == RobobrainStateHandler.robostate['FACEDETECTION']:
            self._queue.put(self.data)
