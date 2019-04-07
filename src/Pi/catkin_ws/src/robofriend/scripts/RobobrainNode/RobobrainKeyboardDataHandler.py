from RobobrainStateHandler import *
import rospy
import sys
import re

# import ros services
from robofriend.srv import SrvTeensySerialData
from robofriend.srv import SrvFaceDrawData
from robofriend.srv import SrvSoundData

# import ros messages
from robofriend.msg import SpeechData

class RobobrainKeyboardDataHandler():

    teensy_methods = {'STOP_MOVING' :   'D 0 0 0',
                      'MOVE_LOOP_FWD' : 'D 255 255 0',
                      'MOVE_LOOP_BCK' : 'D -255 -255 0',
                      'MOVE_LOOP_RYT' : 'D 128 -128 0',
                      'MOVE_LOOP_LFT' : 'D -128 128 0'
                      }

    ENTER =           'enter'
    QUIT =            'quit'
    ESCAPE =          'escape'
    BACKSPACE =       'backspace'
    UP =              'up'
    DOWN =            'down'
    LEFT =            'left'
    RIGHT =           'right'
    INCREASE_SMILE =  ','
    DECREASE_SMILE =  '.'
    PLAY_RANDOM =     '-'

    def __init__(self, sh, event, queue):
        self._idle_event = event
        self._statehandler = sh
        self._queue = queue

        self._speech_buffer = ""
        self._last_say = ""


        # init service to communicate with teensy node
        rospy.wait_for_service('/robofriend/teensy_serial_data')
        self._teensy_request = rospy.ServiceProxy('/robofriend/teensy_serial_data', SrvTeensySerialData)

        # init service to communicate with face node
        rospy.wait_for_service('/robofriend/face')
        self._face_request = rospy.ServiceProxy('/robofriend/face', SrvFaceDrawData)

        # init publisher to communicate with speech node
        self._pub_speech = rospy.Publisher('/robofriend/speech_data', SpeechData, queue_size = 0)
        self._msg_speech = SpeechData()

        # init service to communicate with sound node
        rospy.wait_for_service('/robofriend/sound')
        self._sound_request = rospy.ServiceProxy('/robofriend/sound', SrvSoundData)

    def process_data(self, data):
        self._input_handler(data)

    def _input_handler(self, data):
        if self._statehandler.state == RobobrainStateHandler.robostate["IDLE"]:
            self._idle_event.set()      # set event to stay in IDLE State

            self._process_input_idle_state(data)

            #TODO: Input processing according to the IDLE State

        elif self._statehandler.state == RobobrainStateHandler.robostate['FACEDETECTION']:
            self._queue.put(data.pressed_key)

    def _process_input_idle_state(self, data):

        rospy.logwarn("Pressed Key: %s", data.pressed_key)

        up_down = data.up_down
        pressed_key = data.pressed_key

        if up_down == "up" and pressed_key in ["down", "up", "left", "right"]:
            self._teensy_srv_request(self.teensy_methods["STOP_MOVING"])
        elif up_down == "down":

            if pressed_key == self.ENTER:
                if self._speech_buffer == self.QUIT:
                    sys.exit()
                elif self._speech_buffer:
                    self._publish_speech_message("custom", self._speech_buffer)
                    self._last_say = self._speech_buffer
                self._speech_buffer = ""
            elif pressed_key == self.ESCAPE:
                rospy.loginfo("Clear speech buffer")
                self._speech_buffer = ""
            elif pressed_key == self.BACKSPACE:
                if self._last_say:
                    self._publish_speech_message("custom", self._last_say)

            ########### Teensy drive commands ###########
            elif pressed_key == self.UP:
                self._teensy_srv_request(self.teensy_methods["MOVE_LOOP_FWD"])
            elif pressed_key == self.DOWN:
                self._teensy_srv_request(self.teensy_methods["MOVE_LOOP_BCK"])
            elif pressed_key == self.LEFT:
                self._teensy_srv_request(self.teensy_methods["MOVE_LOOP_LFT"])
            elif pressed_key == self.RIGHT:
                self._teensy_srv_request(self.teensy_methods["MOVE_LOOP_RYT"])

            ########### Face commands ###########
            elif pressed_key == self.INCREASE_SMILE:
                self._face_srv_request("increase")
            elif pressed_key == self.DECREASE_SMILE:
                self._face_srv_request("decrease")

            ########### Sound commands ###########
            elif pressed_key == self.PLAY_RANDOM:
                self._sound_srv_request("random")

            ########### Speech commands ##########
            elif pressed_key == '1':
                self._publish_speech_message("custom", "Hallo, wie gehts?")
            elif pressed_key == '2':
                self._publish_speech_message("custom", "Danke, mir geht es gut.")
            elif pressed_key == '3':
                self._publish_speech_message("custom", "Willst du etwas zum knabbern?")
            elif pressed_key == '4':
                self._publish_speech_message("custom", "Bitte, gerne.")
            elif pressed_key == '5':
                self._publish_speech_message("custom", "Wie heisst du?")
            elif pressed_key == '6':
                self._publish_speech_message("custom", "Ich heisse Robofreund.")
            elif pressed_key == '7':
                self._publish_speech_message("custom", "Mir  ist langweilig.")
            elif pressed_key == '8':
                self._publish_speech_message("custom", "Heute ist ein schoener Tag")
            elif pressed_key == '9':
                self._publish_speech_message(mode = "random")
            elif pressed_key == '0':
                self._publish_speech_message(mode = "bullshit")
            elif re.match('^[a-zA-Z ]$', pressed_key):
                self._speech_buffer += pressed_key
                rospy.loginfo("Actual Speech Buffer: %s", self._speech_buffer)

    def _teensy_srv_request(self, command):
        rospy.loginfo("{%s} - Teensy command from keyboard pressed: %s",
            rospy.get_caller_id(), command)
        self._teensy_request(command, False)

    def _face_srv_request(self, command):
        param = []
        self._face_request(action = command, param = param)

    def _sound_srv_request(self, command):
        self._sound_request(False, command, "", [])



    def _publish_speech_message(self, mode, text = ""):
        self._msg_speech.mode = mode
        self._msg_speech.text = text
        self._pub_speech.publish(self._msg_speech)
        rospy.loginfo("{%s} - Speech published data: {%s}",
            self.__class__.__name__, str(self._msg_speech))
