import os, rospy, subprocess, Queue, random, time
from threading import Lock, Thread
from time import sleep

# import ros messages
from robofriend.msg import SpeechData
from robofriend.msg import WebserverAviStateData

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
        'VOICEINTERACTION'    : 2
    }

    def __init__(self, event, fd, vd, obj, queue):
        self.__state = RobobrainStateHandler.robostate["IDLE"]
        self.__batWasLow = False
        self.__lock = Lock()
        self.__idle_keyb_event = event
        self.__idle_elapse_time = 60       # waits defined sec to change state if no input from webserver and keyboard
        self.__event_wait_time = 1
        self.__keyboard_queue = queue
        self._audiovisual_cnt = 0

        self._kb_choose_mode = None
        self._wb_choose_mode = None

        # init publishers
        self._pub_speech = rospy.Publisher('/robofriend/speech_data', SpeechData, queue_size = 10)
        self._msg_speech = SpeechData()

        # init webserver subscriber
        rospy.Subscriber('/robofriend/web_state_data', WebserverAviStateData, self._webserver_process_data)


        self._fd = fd
        self._vd = vd
        self._obj = obj
        self.__start_thread()

    def __start_thread(self):
        thread = Thread(target = self.__state_handler_thread)
        thread.daemon = True
        thread.start()

    def __state_handler_thread(self):

        previous_interaction_mode = None
        face_familiarity = "no_person"
        person_detected = False
        update_timer_flag = True

        idle_timer = 0
        speak_timer = 0

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

                if update_timer_flag is True:
                    idle_timer = self._system_time_request()
                    speak_timer = self._system_time_request()
                    update_timer_flag = False
                elif self._system_time_request() - speak_timer > 10:
                    self._publish_speech_message("idle")
                    speak_timer = self._system_time_request()

                self.__batWasLow = False
                # speakOnRecharge()
                event_is_set = self.__idle_keyb_event.wait(self.__event_wait_time)
                if event_is_set is True:
                    self.__idle_keyb_event.clear()
                    kb_input = self._get_keyboard_input()
                    if kb_input is not None:
                        self._kb_choose_mode = kb_input
                        self.state = RobobrainStateHandler.robostate["AUDIOVISUAL_INTERACTION"]
                        update_timer_flag = True
                    else:
                        pass
                    idle_timer = self._system_time_request()
                else:
                    if self._system_time_request() - idle_timer > self.__idle_elapse_time:
                        rospy.loginfo("No input within %s seconds therefore change state to AUDIOVISUAL_INTERACTION state!", str(self.__idle_elapse_time))
                        self.state = RobobrainStateHandler.robostate["AUDIOVISUAL_INTERACTION"]
                        update_timer_flag = True
            # ************************* AUTONOM ************************** '''
            elif self.state == RobobrainStateHandler.robostate["AUTONOM"]:
                rospy.loginfo("{%s} - within autonom state", rospy.get_caller_id())
            # ************************** MANUAL ************************** '''
            elif self.state == RobobrainStateHandler.robostate["MANUAL"]:
                rospy.loginfo("{%s} - within manual state", rospy.get_caller_id())
            # *********************** AUDIOVISUAL_INTERACTION ********************** '''
            elif self.state == RobobrainStateHandler.robostate["AUDIOVISUAL_INTERACTION"]:

                rospy.loginfo("{%s} - within audiovisual interaction state", rospy.get_caller_id())
                if self._kb_choose_mode is None and self._wb_choose_mode is None:
                    mode = self._choose_random_interaction_mode(previous_interaction_mode)
                    if mode == "facedetection":
                        previous_interaction_mode = "facedetection"
                        face_familiarity = self._start_face_interaction()
                    elif mode == "objectdetection":
                        rospy.logdebug("Start Objectdetetcion")
                        previous_interaction_mode = "objectdetection"
                        person_detected = self._start_object_interaction()
                    elif mode == "voiceinteraction":
                        rospy.logdebug("Start Voiceinteraction")
                        self._start_voice_interaction(previous_interaction_mode, face_familiarity, person_detected)
                        previous_interaction_mode = "voiceinteraction"
                else:
                    if self._kb_choose_mode == self.interaction["FACEDETECTION"] or self._wb_choose_mode == self.interaction["FACEDETECTION"]:
                        face_familiarity = self._start_face_interaction()
                        previous_interaction_mode = "facedetection"
                    elif self._kb_choose_mode == self.interaction["OBJECTDETECTION"] or self._wb_choose_mode == self.interaction["OBJECTDETECTION"]:
                        person_detected = self._start_object_interaction()
                        previous_interaction_mode = "objectdetection"
                    elif self._kb_choose_mode == self.interaction["VOICEINTERACTION"] or self._wb_choose_mode == self.interaction["VOICEINTERACTION"]:
                        self._start_voice_interaction(previous_interaction_mode, face_familiarity, person_detected)
                        previous_interaction_mode = "voiceinteraction"
                    else:
                        rospy.logwarn("Wrong keyboard input for audio audiovisual interaction!")

                if self._kb_choose_mode is not None or self._wb_choose_mode is not None:
                    self._kb_choose_mode = None
                    self._wb_choose_mode = None
                    self.state = RobobrainStateHandler.robostate["IDLE"]
                elif self._audiovisual_cnt == 2:
                    self._audiovisual_cnt = 0
                    self.state = RobobrainStateHandler.robostate["IDLE"]
                    face_familiarity = None
                    previous_interaction_mode = None
                    person_detected = False
                    update_timer_flag = True
                else:
                    self._audiovisual_cnt += 1
            # ************************** CHARGE ************************** '''
            elif self.state == RobobrainStateHandler.robostate["CHARGE"]:
                rospy.loginfo("{%s} - within charge state", rospy.get_caller_id())
                self.__batWasLow = True
                self._publish_speech_message("battery", "low")
                # TODO: speakBatteryLow()
                # TODO: publish message to speach Node: mode: battery / text = low / recharge
            # ************************** ADMIN *************************** '''
            elif self.state == RobobrainStateHandler.robostate["ADMIN"]:
                rospy.loginfo("{%s} - within admin state", rospy.get_caller_id())
            # ************************* UNKNOWN ************************** '''
            else:
                rospy.logerr("{%s} - robot state unknown", rospy.get_caller_id())

            time.sleep(0.5)

    def _webserver_process_data(self, data):
        rospy.logdebug("{%s} - Message from Webserver received: %s",
            rospy.get_caller_id(), data.state)

        if data.state in ["facedetection", "objectdetection", "voiceinteraction"]:
            if self.state != RobobrainStateHandler.robostate["CHARGE"]:
                self.state = RobobrainStateHandler.robostate["AUDIOVISUAL_INTERACTION"]
                self._wb_choose_mode = self.interaction[data.state.upper()]
                rospy.logdebug("State from Webserver: %s", self._wb_choose_mode)
            else:
                rospy.logwarn("{%s} - No state change possible since Robofriend is in recharge state!",
                    rospy.get_caller_id())
        else:
            rospy.logwarn("{%s} - Invalid State from Webserver received!")

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
                retVal = self.interaction["OBJECTDETECTION"]
            elif keyboard_input == "voiceinteraction":
                retVal = self.interaction["VOICEINTERACTION"]
        except Queue.Empty:
            rospy.logwarn("No keyboard input")
            retVal = None
        return retVal

    def _choose_random_interaction_mode(self, prev_mode):
        mode = ["facedetection", "objectdetection", "voiceinteraction"]
        if prev_mode in mode:
            mode.remove(prev_mode)
        retVal = random.choice(mode)
        return retVal

    def _start_face_interaction(self):
        rospy.logwarn("Start Facedetetcion interaction state")
        face_familiarity = self._fd._start_facedetection()
        rospy.logdebug("Detected Face: %s", face_familiarity)
        return face_familiarity

    def _start_object_interaction(self):
        rospy.logwarn("Start Object detection interaction state")
        person_detected = self._obj._start_persondetection()
        rospy.logwarn("Person Detected: %s", str(person_detected))
        return person_detected

    def _start_voice_interaction(self, prev_mode = None, face_familiarity = "no_person", person_detected = False):
        rospy.logwarn("Start Voice detection interaction state")
        self._vd._start_voiceinteraction(prev_mode, face_familiarity, person_detected)

    def _system_time_request(self):
        return time.time()

    def _publish_speech_message(self, mode, text = ""):
        self._msg_speech.mode = mode
        self._msg_speech.text = text
        self._pub_speech.publish(self._msg_speech)
        rospy.logdebug("{%s} - Speech published data: {%s}",
            self.__class__.__name__, str(self._msg_speech))
