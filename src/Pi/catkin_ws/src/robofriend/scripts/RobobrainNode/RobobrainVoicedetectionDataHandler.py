from RobobrainStateHandler import *
from time import *
import rospy
import Queue

#from urllib.request import urlopen     # python3
from urllib2 import urlopen             # python2

# import ros services
from robofriend.srv import SrvVoiceHotwordActivationData

# import ros messages
from robofriend.msg import SpeechData
from robofriend.msg import VoiceData

class RobobrainVoicedetectionDataHandler():

    def __init__(self, queue):
        self._keyboard_queue = queue
        self._voice_hotword_request = rospy.ServiceProxy('/robofriend/voicehotword', SrvVoiceHotwordActivationData)
        self._elapse_time = 10
        self._intent = ""
        self._slot = ""

        # init publishers
        self._pub_speech = rospy.Publisher('/robofriend/speech_data', SpeechData, queue_size = 10)
        self._msg_speech = SpeechData()

    def process_data(self, data):
        rospy.logwarn("{%s} -  process_data : Message received from Voice Detection Node: %s",
            self.__class__.__name__, data)
        self._intent = data.intent
        self._slot = data.slot

        self._input_handler(data)

    def _start_voiceinteraction(self, prev_mode, face_familiarity, person_detect):
        self._smart_home_interaction()
        # if prev_mode == "facedetection" or prev_mode == "objectdetection":
        #     if face_familiarity == "no_person" or person_detect is False:
        #         rospy.logwarn("{%s} - Previous Mode: %s, Face Familiarity: %s, Person detected: %s", self.__class__.__name__, prev_mode. face_familiarity, person_detect)
        #         rospy.logwarn("{%s} - Start DOA", self.__class__.__name__)
        #         #TODO: add Speech
        #     elif face_familiarity != "unknown":
        #         rospy.logwarn("{%s} - Start smart home interaction", self.__class__.__name__)
        #     elif person_detect is True:
        #         rospy.logwarn("{%s} - Start smart home interaction", self.__class__.__name__)
        #     elif face_familiarity == "unknown":
        #         if _choose_random_interaction_mode() == "doa":
        #             rospy.logwarn("{%s} - Familiarity: unknown, Start DOA", self.__class__.__name__)
        #         else:
        #             rospy.logwarn("{%s} - Start smart home interaction", self.__class__.__name__)
        # elif prev_mode is None:
        #     rospy.logwarn("{%s} - Previous Mode: %s, Face Familiarity: %s, Person detected: %s", self.__class__.__name__, prev_mode. face_familiarity, person_detect)
        #     rospy.logwarn("{%s} - Start DOA", self.__class__.__name__)
        # else:
        #     rospy.logwarn("{%s} - Last Else", self.__class__.__name__)
        #     pass

    def _smart_home_interaction(self):
        yes_no = None

        self._publish_speech_message("custom", "Da wir uns kennen hast du die volle kontrolle uber mein zu Hause")
        sleep(5)
        while yes_no is not False:
            self._publish_speech_message("custom", "Was mochtest du in meiner Wohnunng steuern")
            sleep(5)
            response = self._voice_hotword_request(True)
            if response.response is True:
                rospy.logdebug("Response is True")
                self._evaluate_voice_inputs()
            sleep(1)
            self._publish_speech_message("custom", "Mochtest du weiter machen tippe ja oder nein ein")
            sleep(3)
            yes_no = self._yes_no_keyboard_request()


    def _evaluate_voice_inputs(self):
        try:
            vc_input =rospy.wait_for_message('/robofriend/voice_data', VoiceData, timeout = self._elapse_time)
            rospy.logdebug("{%s} - evaluate_voice_inputs : Received message: %s", self.__class__.__name__, vc_input)
            start_time = self._time_request()
            sep_mes = vc_input.slots.split("/")
            if vc_input.intent == "lights":
                if sep_mes[1] == "on":
                    if sep_mes[0] == "living room":
                        rospy.logdebug("Living room lights on!\n")
                        #urlopen("http://172.22.0.166:8081/rest/runtime/model/components/67-111-109-109-97-110-100-73-110-112-117-116-/ports/105-110-/data/64-75-78-88-58-49-49-47-48-47-48-44-49-46-48-48-49-44-111-110-")
                    elif sep_mes[0] == "kitchen":
                        rospy.logdebug("Kitchen lights on!\n")
                        #urlopen("http://172.22.0.166:8081/rest/runtime/model/components/67-111-109-109-97-110-100-73-110-112-117-116-/ports/105-110-/data/64-75-78-88-58-49-49-47-48-47-56-44-49-46-48-48-49-44-111-110-")
                elif sep_mes[1] == "off":
                    if sep_mes[0] == "living room":
                        rospy.logdebug("Living room lights off!\n")
                        #urlopen("http://172.22.0.166:8081/rest/runtime/model/components/67-111-109-109-97-110-100-73-110-112-117-116-/ports/105-110-/data/64-75-78-88-58-49-49-47-48-47-48-44-49-46-48-48-49-44-111-102-102-")
                    elif sep_mes[0] == "kitchen":
                        rospy.logdebug("Kitchen lights off!\n")
                                #urlopen("http://172.22.0.166:1ROSException/rest/runtime/model/components/67-111-109-109-97-110-100-73-110-112-117-116-/ports/105-110-/data/64-75-78-88-58-49-49-47-48-47-56-44-49-46-48-48-49-44-111-102-102-")
        except rospy.ROSException:
            rospy.logdebug("{%s} - Timeout occured within {%s} seconds!\n"
                , self.__class__.__name__, self._elapse_time)
            self._publish_speech_message("custom", "Ich habe dich nicht verstanden")
            return False, None

    def _choose_random_interaction_mode(self):
        mode = ["doa", "smart_home"]
        retVal = random.choice(mode)
        return retVal

    def _yes_no_keyboard_request(self):
        rospy.logdebug("{%s} - Waiting for yes or no!\n", self.__class__.__name__)
        retVal, keyboard_input = self._evaluate_keyboard_inputs()
        if retVal != True:
            return False
        else:
            keyboard_input.lower()
            if keyboard_input == "ja" or keyboard_input == 'yes':
                return True
            elif keyboard_input == "nein" or keyboard_input == 'no':
                return False

    def _evaluate_keyboard_inputs(self):
        keyboard_input = ""
        retString = ""
        start_time = self._time_request()
        try:
            while self._time_request() - start_time < self._elapse_time:
                keyboard_input = self._keyboard_queue.get(timeout = self._elapse_time)
                if keyboard_input == "enter":
                    rospy.logdebug("{%s} - Enter is pressed!\n", self.__class__.__name__)
                    return True, retString
                elif keyboard_input == "backspace":
                    retString = retString[:len(retString) - 1]
                    rospy.logdebug("{%s} - String after backspace: {%s}\n",
                        self.__class__.__name__, retString)
                elif keyboard_input == "space":
                    retString += " "
                else:
                    retString += keyboard_input
                    rospy.logdebug("{%s} - String: {%s}\n",
                        self.__class__.__name__, retString)
            else:
                rospy.logdebug("{%s} - Loop stopped since enter button not pressed!\n",
                    self.__class__.__name__)
                self._publish_speech_message("custom", "Du hast nicht enter getippt!")
                return False, None
        except Queue.Empty:
            rospy.logdebug("{%s} - Timeout occured within {%s} seconds!\n"
                , self.__class__.__name__, self._elapse_time)
            self._publish_speech_message("custom", "Du warst mit der eingabe zu langsam")
            return False, None

    def _time_request(self):
        return time()

    def _publish_speech_message(self, mode, text = None):
        self._msg_speech.mode = mode
        self._msg_speech.text = text
        self._pub_speech.publish(self._msg_speech)
        rospy.logdebug("{%s} - Speech published data: {%s}",
            self.__class__.__name__, str(self._msg_speech))
