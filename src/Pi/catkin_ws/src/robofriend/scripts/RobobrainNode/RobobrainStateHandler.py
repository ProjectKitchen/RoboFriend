import os, rospy, subprocess, Queue, random

from threading import Lock, Thread
from time import sleep

class RobobrainStateHandler():

    robostate = {
        'ADMIN' : 0, \
        'CHARGE' : 1, \
        'AUDIOVISUAL_INTERACTION' : 2, \
        'MANUAL' : 3, \
        'AUTONOM' : 4, \
        'IDLE' : 5, \
        'SHUTDOWN' : 6
    }

    interaction = {
        'FACEDETECTION'     : 0, \
        'OBJECTDETECTION'   : 1, \
        'VOICEDETECTION'    : 2
    }

    def __init__(self, event, fd, vd, queue):
        self.__state = RobobrainStateHandler.robostate["IDLE"]
        self.__batWasLow = False
        self.__lock = Lock()
        self.__idle_keyb_event = event
        self.__idle_elapse_time = 10         # waits defined sec to change state if no input from webserver and keyboard
        self.__keyboard_queue = queue
        self._audiovisual_cnt = 0

        self._fd = fd
        self._vd = vd
        self.__start_thread()

    def __start_thread(self):
        thread = Thread(target = self.__state_handler_thread)
        thread.daemon = True
        thread.start()

    def __state_handler_thread(self):

        previous_interaction_mode = None
        kb_choose_mode = None
        face_familiarity = "known"

        while True:
            # ************************* SHUTDOWN *************************
            if self.state == RobobrainStateHandler.robostate["SHUTDOWN"]:
                rospy.logwarn("{%s} - within shutdown state", rospy.get_caller_id())
                # TODO: speakBatteryShutdown()
                sleep(5)
                subprocess.call(["sudo", "init", "0"])
            # *************************** IDLE *************************** '''
            elif self.state == RobobrainStateHandler.robostate["IDLE"]:
                rospy.loginfo("{%s} - within idle state", rospy.get_caller_id())
                self.__batWasLow = False
                # speakOnRecharge()
                event_is_set = self.__idle_keyb_event.wait(self.__idle_elapse_time)
                if event_is_set == True:
                    rospy.logdebug("{%s} - Input from keyboard node received therefore stay in IDLE Stat!")
                    self.__idle_keyb_event.clear()
                    kb_input = self._get_keyboard_input()
                    if kb_input is not None:
                        kb_choose_mode = kb_input
                        self.state = RobobrainStateHandler.robostate["AUDIOVISUAL_INTERACTION"]
                    else:
                        pass
                else:
                    rospy.logdebug("No input within %s seconds therefore change state to AUDIOVISUAL_INTERACTION state!", str(self.__idle_elapse_time))
                    self.state = RobobrainStateHandler.robostate["AUDIOVISUAL_INTERACTION"]
            # ************************* AUTONOM ************************** '''
            elif self.state == RobobrainStateHandler.robostate["AUTONOM"]:
                rospy.loginfo("{%s} - within autonom state", rospy.get_caller_id())
            # ************************** MANUAL ************************** '''
            elif self.state == RobobrainStateHandler.robostate["MANUAL"]:
                rospy.loginfo("{%s} - within manual state", rospy.get_caller_id())
            # *********************** AUDIOVISUAL_INTERACTION ********************** '''
            elif self.state == RobobrainStateHandler.robostate["AUDIOVISUAL_INTERACTION"]:
                rospy.loginfo("{%s} - within audiovisual interaction state", rospy.get_caller_id())
                if kb_choose_mode == None:
                    mode = self._choose_random_interaction_mode(previous_interaction_mode)

                    if mode == "facedetection":
                        previous_interaction_mode = "facedetection"
                        self._start_face_interaction()
                    elif mode == "objectdetection":
                        rospy.logdebug("Start Objectdetetcion")
                        previous_interaction_mode = "objectdetection"
                        self._start_object_interaction()
                    elif mode == "voicedetection":
                        rospy.logdebug("Start Voicedetection")
                        previous_interaction_mode = "voicedetection"
                        self._start_voice_interaction(previous_interaction_mode, face_familiarity)
                else:
                    if kb_choose_mode == self.interaction["FACEDETECTION"]:
                        previous_interaction_mode = "facedetection"
                        self._start_face_interaction()
                    elif kb_choose_mode == self.interaction["OBJECTDETECTION"]:
                        previous_interaction_mode = "objectdetection"
                        self._start_object_interaction()
                    elif kb_choose_mode == self.interaction["VOICEDETECTION"]:
                        previous_interaction_mode = "voicedetection"
                        self._start_voice_interaction(previous_interaction_mode, face_familiarity)
                    else:
                        rospy.logwarn("Wrong keyboard input for audio audiovisual interaction!")
                    kb_choose_mode = None

                if self._audiovisual_cnt == 2:
                    self.state = RobobrainStateHandler.robostate["IDLE"]
                    self._audiovisual_cnt = 0
                else:
                    self._audiovisual_cnt += 1

                sleep(5)

            # ************************** CHARGE ************************** '''
            elif self.state == RobobrainStateHandler.robostate["CHARGE"]:
                rospy.loginfo("{%s} - within charge state", rospy.get_caller_id())
                self.__batWasLow = True
                # TODO: speakBatteryLow()
                # TODO: publish message to speach Node: mode: battery / text = low / recharge
            # ************************** ADMIN *************************** '''
            elif self.state == RobobrainStateHandler.robostate["ADMIN"]:
                rospy.loginfo("{%s} - within admin state", rospy.get_caller_id())
            # ************************* UNKNOWN ************************** '''
            else:
                rospy.logerr("{%s} - robot state unknown", rospy.get_caller_id())

    @property
    def state(self):
        self.__lock_acquire()
        retVal = None
        retVal = self.__state
        self.__lock_release()
        return retVal

    @state.setter
    def state(self, value):
        self.__lock_acquire()
        self.__state = value
        self.__lock_release()

    def __lock_acquire(self):
        self.__lock.acquire()

    def __lock_release(self):
        self.__lock.release()

    def _get_keyboard_input(self):
        retVal = None
        try:
            keyboard_input = self.__keyboard_queue.get(timeout = 1)     # waits for an input from keyboard node
            if keyboard_input == "facedetection":
                retVal = self.interaction["FACEDETECTION"]
            elif keyboard_input == "objectdetection":
                retVal = self.interaction["OBJECTDETETCION"]
            elif keyboard_input == "voicedetection":
                retVal = self.interaction["VOICEDETECTION"]
        except Queue.Empty:
            rospy.logdebug("No keyboard input")
            retVal = None
        return retVal

    def _choose_random_interaction_mode(self, prev_mode):
        mode = ["facedetection", "objectdetection", "voicedetection"]
        if prev_mode in mode:
            mode.remove(prev_mode)
        retVal = random.choice(mode)
        return retVal

    def _start_face_interaction(self):
        rospy.logdebg("Start Facedetetcion")

    def _start_object_interaction(self):
        rospy.logdebug("Start Object detection")

    def _start_voice_interaction(self, prev_mode, face_familiarity):
        rospy.logdebug("Start Voice detection")
