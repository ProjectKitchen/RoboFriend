#from RobobrainStateHandler import *

from RobobrainStateHandler import *

from threading import *
from time import *
#from queue import *
from Queue import *
import rospy
import traceback
import urlib2

# import ros service
from robofriend.srv import SrvFaceRecordData
from robofriend.srv import SrvFaceDatabaseData
from robofriend.srv import SrvVoiceHotwordActivationData

# import ros messages
from robofriend.msg import SpeechData
from robofriend.msg import LedEarsData
from robofriend.msg import ServoCamData

class RobobrainFacedetectionDataHandler():

    def __init__(self, sh, queue, vc_queue):
        self._top = 0
        self._right = 0
        self._bottom = 0
        self._left = 0
        self._name = None
        self._face_node_started = False

        self._statehandler = sh
        self._keyboard_queue = queue
        self._vc_queue = vc_queue

        # amount of recorded pictures
        self.__pic_record = 10

        self._elapse_time = 30

        self.__record_pic_speech = {1  : "Erstes", \
                                    2  : "Zweites", \
                                    3  : "Drittes", \
                                    4  : "Viertes", \
                                    5  : "Funftes", \
                                    6  : "Sechstes", \
                                    7  : "Siebentes", \
                                    8  : "Achtes", \
                                    9  : "Neuntes", \
                                    10 : "Zehntes"}

        # init publishers
        self._pub_speech = rospy.Publisher('/robofriend/speech_data', SpeechData, queue_size = 10)
        self._pub_led_ears = rospy.Publisher('/robofriend/led_ears_data', LedEarsData, queue_size = 10)
        self._pub_servo_cam = rospy.Publisher('/robofriend/servo_cam_data', ServoCamData, queue_size = 10)

        self._msg_speech = SpeechData()
        self._msg_led_ears = LedEarsData()
        self._msg_servo_cam = ServoCamData()
        self._search_new_face = Event()

        try:
            rospy.wait_for_service('/robofriend/facerecord', timeout = self._elapse_time)
            rospy.wait_for_service('/robofriend/facedatabase', timeout = self._elapse_time)
        except rospy.ROSException:
            rospy.logwarn("{%s} - Facedetection-Node was not able to start therefore no facerecognition possible",
                self.__class__.__name__)
            self._face_node_started = False
        else:
            rospy.logdebug("{%s} - Facedection Node started!", self.__class__.__name__)
            self.__facerecord_request = rospy.ServiceProxy('/robofriend/facerecord', SrvFaceRecordData)
            self.__facedatabase_request = rospy.ServiceProxy('/robofriend/facedatabase', SrvFaceDatabaseData)
            self._voice_hotword = rospy.ServiceProxy('/robofriend/voicehotword', SrvVoiceHotwordActivationDatar)

            self._face_node_started = True

        self.__start_thread()

    def process_data(self, data):
        rospy.logdebug("{%s} - Received message: {%s}",
            self.__class__.__name__, str(data))
        if self.__is_facesearching_activated():
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
            while self._statehandler.state == RobobrainStateHandler.robostate["FACEDETECTION"]:
                if self._face_node_started is False:
                    rospy.logdebug("{%s} - Facedection Node not started therefore change to IDLE - State", self.__class__.__name__)
                    self._statehandler.state = RobobrainStateHandler.robostate["IDLE"]
                elif self._face_node_started is True:
                    face_detectded, face_grade = self.__face_search()
                    self.__stop_searching_new_face()
                    if  face_detectded is True:
                        rospy.logdebug("{%s} - Face detected! Further steps are activated!\n", self.__class__.__name__)
                        self.__publish_speech_message("custom", "Ich habe jemanden gefunden")
                        rospy.logwarn(face_grade)
                        face_grade .lower()
                        if face_grade != "unknown":
                            self.__known_face_speech(face_grade)
                            if self.__take_picture_known_face(face_grade) is True:
                                self.__create_database()

                            # start voice interaction
                            while True:
                                self._voice_interaction()

                            ##########################################################
                            #TODO: Do mething in case of known face
                            ##########################################################

                        elif face_grade == "unknown":
                            self.__unknown_face_speech()
                            if self.__yes_no_keyboard_request() is True:
                                rospy.logdebug("{%s} - Start recording Pictures!\n", self.__class__.__name__)
                                if self.__start_recording_faces() is True:    # start recording faces
                                    rospy.loginfo("{%s} - Faces are recorded!\n", self.__class__.__name__)

                                    self.__create_database()    #TODO: stop when recording new faces


                                elif self.__start_recording_faces() is False:
                                    rospy.logwarn("{%s} - Recording new faces failed!\n", self.__class__.__name__)
                                    break
                            else:
                                self.__publish_speech_message("custom", "Ich darf mit fremden Leuten nicht reden")
                                rospy.logdebug("{%s} - Recording Pictures not allowed! Start searching new face\n", self.__class__.__name__)
                                break
                    elif face_detectded is False:
                        # self.__stop_searching_new_face()
                        self.__publish_speech_message("custom", "Keine menschensseele hier!")
                        rospy.logdebug("{%s} -  No Face detected within {%s} seconds! Change State to IDLE State\n", self.__class__.__name__, str(self._elapse_time))
                        self._statehandler.state = RobobrainStateHandler.robostate["IDLE"]
                        break
            else:
                sleep(1)

    def __face_search(self):
        self.__publish_speech_message("custom", "Ich schaue mich dann mal nach Menschen um!")
        self.__start_searchig_new_face()
        self.__publish_servo_cam_message("min")
        start_time = self.__time_request()
        while self.__time_request() - start_time < self._elapse_time:
            self.__publish_servo_cam_message(diff = 10)
            if self._name is not None:
                return True, self._name
            sleep(2)
        else:
            print("Time for finding a face expired! Change state to IDLE State")
            return False, None

    def __time_request(self):
        return time()

    def __start_searchig_new_face(self):
        print("[INFO] Set event to start searching new face!\n")
        self._search_new_face.set()

    def __stop_searching_new_face(self):
        print("[INFO] Clear event to stop searching new face!\n")
        self._search_new_face.clear()
        self._top = 0
        self._right = 0
        self._bottom = 0
        self._left = 0
        self._name = None

    def __is_facesearching_activated(self):
        return self._search_new_face.is_set()

    def __known_face_speech(self, name):
        self.__publish_led_ears_message(rgb_color = [0, 15, 0]) # to flush the ears in green
        self.__publish_speech_message("custom", "Ich kenne dich!")
        self.__publish_speech_message("custom", "Du bist " + name)

    def __unknown_face_speech(self):
        self.__publish_led_ears_message(rgb_color = [15, 0, 0]) # to flush the ears in red
        self.__publish_speech_message("custom", "Ich kenne dich nicht!")
        self.__publish_speech_message("custom", "Ich mochte dich kennen lernen. Darf ich Bilder von dir aufnehmen?")

    def __take_picture_known_face(self, name):
        runFlag = True
        cnt = 0

        self.__publish_speech_message("custom", "Magst du weitere Bilder aufnehmen?")
        self.__publish_speech_message("custom", "Gebe ja oder nein ein?")
        if self.__yes_no_keyboard_request() is True:
            while runFlag is True:
                if self.__take_pictures(name) is True:
                    cnt += 1
                    self.__publish_speech_message("custom", "Fur weiteres Bild ja eingeben oder nein um es zu stopen")
                else:
                    return False
                runFlag = self.__yes_no_keyboard_request()
            else:
                rospy.loginfo("{%s} - %s new pictures are recorded!\n",
                    self.__class__.__name__, str(cnt))
                return True
        else:
            rospy.loginfo("{%s} - Person rejected to record new pictures!\n",
                self.__class__.__name__)
            return False

    def __yes_no_keyboard_request(self):
        #TODO: replace with voice detection
        rospy.logdebug("{%s} - Waiting for yes or no!\n", self.__class__.__name__)
        retVal, keyboard_input = self.__evaluate_keyboard_inputs()
        if retVal != True:
            return False
        else:
            keyboard_input.lower()
            if keyboard_input == "ja" or keyboard_input == 'yes':
                return True
            elif keyboard_input == "nein" or keyboard_input == 'no':
                return False

    def __evaluate_keyboard_inputs(self):
        keyboard_input = ""
        retString = ""
        start_time = self.__time_request()
        try:
            while self.__time_request() - start_time < self._elapse_time:
                keyboard_input = self._keyboard_queue.get(timeout = self._elapse_time)
                if keyboard_input == "enter":
                    rospy.loginfo("{%s} - Enter is pressed!\n", self.__class__.__name__)
                    return True, retString
                elif keyboard_input == "backspace":
                    retString = retString[:len(retString) - 1]
                    rospy.loginfo("{%s} - String after backspace: {%s}\n",
                        self.__class__.__name__, retString)
                elif keyboard_input == "space":
                    retString += " "
                else:
                    retString += keyboard_input
                    rospy.loginfo("{%s} - String: {%s}\n",
                        self.__class__.__name__, retString)
            else:
                rospy.loginfo("{%s} - Loop stopped since enter button not pressed!\n",
                    self.__class__.__name__)
                self.__publish_speech_message("custom", "Du hast nicht enter getippt!")
                return False, None
        except Empty:
            rospy.loginfo("{%s} - Timeout occured within {%s} seconds!\n"
                , self.__class__.__name__, self._elapse_time)
            self.__publish_speech_message("custom", "Du warst mit der eingabe zu langsam")
            return False, None

        # send request to start recording faces and wait until all faces are recorded
    def __start_recording_faces(self):
        retVal = False
        recording_response = None
        name_lastname = None

        self.__publish_speech_message("custom", "Bitte Namen eintippen!")
        retVal, name = self.__evaluate_keyboard_inputs()
        if retVal is True:
            name = name.lower()
            check_database = self.__check_name_database(name)
            if check_database is None:          # in case of an error
                rospy.logwarn("{%s} - Error while checking name in database!",
                    self.__class__.__name__)
                retVal = False
                return retVal

            if check_database is False:       # no directory available with this name => therefore create new directory
                if self.__take_picture_new_face(name) is False:
                    retVal = False
                    return retVal
                else:
                    rospy.logdebug("{%s} - Recording new {%s} faces finished!",
                        self.__class__.__name__, str(self.__pic_record))
                    retVal = True
                    return retVal
            elif check_database is True:   # a directory with the entered name already exists
                self.__publish_speech_message("custom", "Ich kenne bereits eine Person mit diesem Namen!")
                self.__publish_speech_message("custom", "Bitte gebe deinen Namen mit deinem Nachnamen ein!")

                while name_lastname is None or name_lastname == name:
                    retVal, name_lastname = self.__evaluate_keyboard_inputs()
                    if name_lastname == name:
                        self.__publish_speech_message("custom", "Bitte unterschiedlichen Namen eingeben!")
                else:
                    rospy.loginfo("{%s} - Different name entered!", self.__class__.__name__)
                    if self.__take_picture_new_face(name_lastname) is False:
                        retVal = False
                        return retVal
                    else:
                        rospy.logdebug("{%s} - Recording new {%s} faces finished!",
                            self.__class__.__name__, str(self.__pic_record))
                        retVal = True
                        return retVal
        elif retVal is False:
            rospy.logdebug("{%s} - No name entered", self.__class__.__name__)
            self.__publish_speech_message("custom", "Falschen Namen einegeben!")
            return retVal

    def __take_picture_new_face(self, name):
        retVal = False

        self.__publish_speech_message("custom", "Hallo {}!".format(name))
        self.__publish_speech_message("custom", "Ich starte mit der aufnahme von zehn Bildern!")
        for cnt in range(1, self.__pic_record + 1):
            if self.__take_pictures(name) is False:
                retVal = False
                return retVal
            else:
                 try:
                    self.__publish_speech_message("custom", "{} Bild aufgenommen!".format(self.__record_pic_speech[cnt]))
                 except KeyError:
                     self.__publish_speech_message("custom", "Weiteres Bild aufgenommen!")
                 except Exception:
                     traceback.print_exc()
                     raise
            sleep(2)
        rospy.logdebug("{%s} - Recording new faces finished!\n",
            self.__class__.__name__)
        retVal = True
        self.__publish_speech_message("custom", "Bin mit er aufnahme fertig")
        return retVal


    def __take_pictures(self, name):
        enter = None
        retVal = False
        rospy.loginfo("{%s} - Request for recording new picture!\n",
            self.__class__.__name__)

        self.__publish_led_ears_message(rgb_color = [15, 0, 0]) # to flush the ears in red
        self.__publish_speech_message("custom", "Enter tippen wenn das Foto aufgenommen werden soll")
        while enter is not "":
            retVal, enter = self.__evaluate_keyboard_inputs()
            rospy.loginfo("{%s} - Entered Input: {%s}\n",
                self.__class__.__name__, str(enter))
            if retVal is False:
                self.__publish_speech_message("custom", "Bitte enter eingeben")
            elif enter is not "":
                self.__publish_speech_message("custom", "Bitte nur enter eingeben")
        else:
            rospy.loginfo("{%s} - Sending request and waiting for response!\n",
                self.__class__.__name__)
            recording_response = self.__facerecord_request(check_name = False, name = name)
            if recording_response.picture_taken is True:
                self.__publish_led_ears_message(rgb_color = [0, 15, 0])
                rospy.loginfo("{%s} - New Picture reorded!", self.__class__.__name__)
                retVal = True
                return retVal
            elif recording_response.picture_taken is False:
                rospy.logerr("{%s} - Error while recording new picture",
                    self.__class__.__name__)
                retVal = False
                return retVal

    def __check_name_database(self, name):
        retVal = None
        try:
            response = self.__facerecord_request(check_name = True, name = name)
        except rospy.ServiceException:
                rospy.logwarn("{%s} - Service call failed!", self.__class__.__name__)
                retVal = None                       # return None in case of an error
        if response.check_name_resp is False:
            rospy.loginfo("{%s} - No directory with that name!\n", self.__class__.__name__)
            return False                             # return False if no directory with this name exists
        elif response.check_name_resp is True:
            rospy.loginfo("{%s} - Directory with this name in database!\n", self.__class__.__name__)
            return True                             # return True if a directory with this name already exists

    def __create_database(self):
        retVal = False

        self.__publish_speech_message("custom", "Ich versuche dein Gesicht und dein Namen zu merken")
        self.__publish_speech_message("custom", "Verzeich mir aber das kann lange dauern")
        try:
            self.__publish_led_ears_message(random = "on")
            response = self.__facedatabase_request(True)
            if response.response == True:
                print("[INFO] {} - Creating Database is finished!\n".format(self.__class__.__name__))
                self.__publish_speech_message("custom", "Bin mit dem merken fertig!")
                self.__publish_speech_message("custom", "Willkommen im meinem Freundeskreis")
                retVal = True
        except rospy.ServiceException:
            print("[INFO] {} - Service call failed!\n".format(__class__.__name__))
            retVal = False
        except Exception :
            traceback.print_exc()
        finally:
            self.__publish_led_ears_message(random = "off")
            if retVal is True:
                rospy.logdebug("{%s} - New Database created!\n", self.__class__.__name__)
            return retVal

    def _voice_interaction(self):
        yes_no = ""

        self.__publish_speech_message("custom", "Da wir uns kennen hast du die volle kontrolle uber mein zu Hause")
        while yes_no is not "nein":
            self.__publish_speech_message("custom", "Was möchtest du in meiner Wohnung steuern")
            response = self._voice_hotword(True)
            if response is True:
                self.__evaluate_voice_inputs()

    def __evaluate_voice_inputs(self):
        try:
            while self.__time_request() - start_time < self._elapse_time:
                vc_input = self._vc_queue.get(timeout = self._elapse_time)
                rosp.logwarn("Voiceeeee intent received")

                sep_mes = vc_input.split("/")
                #######################################
                if vc_input.intent == "lights":
                    if sep_mes[1] == "on":
                        if sep_mes[0] == "living_room":
                            urlib2.urlopen("http://172.22.0.166:8081/rest/runtime/model/components/67-111-109-109-97-110-100-73-110-112-117-116-/ports/105-110-/data/64-75-78-88-58-49-49-47-48-47-48-44-49-46-48-48-49-44-111-110-")
                    else sep_mes[1] == "off":
                        urlib2.urlopen("http://172.22.0.166:8081/rest/runtime/model/components/67-111-109-109-97-110-100-73-110-112-117-116-/ports/105-110-/data/64-75-78-88-58-49-49-47-48-47-48-44-49-46-48-48-49-44-111-102-102-")

            else:
                rospy.loginfo("{%s} - Loop stopped since nothing said!\n",
                    self.__class__.__name__)
                self.__publish_speech_message("custom", "Du hast nicht enter getippt!")
                return False, None
        except Empty:
            rospy.loginfo("{%s} - Timeout occured within {%s} seconds!\n"
                , self.__class__.__name__, self._elapse_time)
            self.__publish_speech_message("custom", "Du warst mit der eingabe zu langsam")
            return False, None

    def __publish_speech_message(self, mode, text = None):
        self._msg_speech.mode = mode
        self._msg_speech.text = text
        self._pub_speech.publish(self._msg_speech)
        rospy.logdebug("{%s} - Speech published data: {%s}",
            self.__class__.__name__, str(self._msg_speech))

    def __publish_led_ears_message(self, random = "", repeat_num = [0, 0], rgb_color = []):
        self._msg_led_ears.random = random
        self._msg_led_ears.repeat_num = repeat_num
        self._msg_led_ears.rgb_color = rgb_color
        self._pub_led_ears.publish(self._msg_led_ears)
        rospy.logdebug("{%s} - Led Ears published data: {%s}",
            self.__class__.__name__, str(self._msg_led_ears))

    def __publish_servo_cam_message(self, max_min = "", diff = 0):
        self._msg_servo_cam.max_min = max_min
        self._msg_servo_cam.diff = diff
        self._pub_servo_cam.publish(self._msg_servo_cam)
        rospy.logdebug("{%s} -  Servo Camera published data: {%s}",
            self.__class__.__name__, str(self._msg_servo_cam))
