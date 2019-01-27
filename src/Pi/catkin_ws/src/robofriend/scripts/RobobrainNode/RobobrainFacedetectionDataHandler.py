#from RobobrainStateHandler import *

from RobobrainNode.RobobrainStateHandler import *

from threading import *
from time import *
from queue import *
#from Queue import *
import rospy

# import ros service
from robofriend.srv import SrvFaceRecordData

# import ros messages
from robofriend.msg import SpeechData
from robofriend.msg import LedEarsData
from robofriend.msg import ServoCamData

class RobobrainFacedetectionDataHandler():

    def __init__(self, sh, queue):
        self._top = 0
        self._right = 0
        self._bottom = 0
        self._left = 0
        self._name = None

        self._statehandler = sh
        self._keyboard_queue = queue

        # init publishers
        self._pub_speech = rospy.Publisher('T_SPEECH_DATA', SpeechData, queue_size = 10)
        self._pub_led_ears = rospy.Publisher('T_LED_EARS_DATA', LedEarsData, queue_size = 10)
        self._pub_servo_cam = rospy.Publisher('T_SERVO_CAM_DATA', ServoCamData, queue_size = 10)

        self._msg_speech = SpeechData()
        self._msg_led_ears = LedEarsData()
        self._msg_servo_cam = ServoCamData()

        self.__start_thread()
        self._elapse_time = 30
        self._search_new_face = Event()

        ##################################
        # uncomment when Facedetection Node is running, otherwise service waits
        #rospy.wait_for_service('/robofriend/facerecord')
        ##################################

        # amount of recorded pictures
        self.__pic_record = 10

        record_pic_speech = {1  : "Erstes", \
                             2  : "Zweites", \
                             3  : "Drittes", \
                             4  : "Viertes", \
                             5  : "Fuenftes", \
                             6  : "Sechstes", \
                             7  : "Siebentes", \
                             8  : "Achtes", \
                             9  : "Neuntes", \
                             10 : "Zehnets"}

    def process_data(self, data):
        print("[INFO] {} - Received message: {} ".format(self.__class__.__name__, data))
        #if self._actual_state == self._facedetection_states["FACE_SEARCH"] and self.__is_facesearching_activated():
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
                face_detectded, face_grade = self.__face_search()
                self.__stop_searching_new_face()
                if  face_detectded is True:
                    print("Face detected! Further steps are activated!\n")
                    self.__publish_speech_message("custom", "Ich habe jemanden gefunden")
                    if face_grade != "unknown":
                        self.__known_face_speech(face_grade)
                        #TODO: Do something in case of a known face
                    elif face_grade == "unknown":
                        self.__unknown_face_speech()
                        if self.__yes_no_keyboard_request() is True:
                            print("[INFO] {} - Start recording Pictures!\n".format(self.__class__.__name__))
                            if self.__start_recording_faces() is True:    # start recording faces
                                print("[INFO] {} - Pictures are recorded!\n".format(self.__class__.__name__))
                                #TODO: Do something when recording new face is finished
                            else:
                                #TODO: When no name is entered!!
                                print("[INFO] {} - No mane entered!\n".format(self.__class__.__name__))
                                break
                        else:
                            print("[INFO] {} - Recording Pictures not allowed! Start searching new face\n".format(self.__class__.__name__))
                            break
                elif face_detectded is False:
                    # self.__stop_searching_new_face()
                    self.__publish_speech_message("custom", "Keine menschensseele hier!")
                    print("[INFO] No Face detected within {} seconds! Change State to IDLE State\n".format(self._elapse_time))
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
        self.__publish_speech_message("custom", "Ich darf mit fremden Leuten nicht reden")
        self.__publish_speech_message("custom", "Darf ich Bilder von dir aufnehmen?")

    def __yes_no_keyboard_request(self):
        #TODO: replace with voice detection
        print("[INFO] Waiting for yes or no!\n")
        retVal, keyboard_input = self.__evaluate_keyboard_inputs()
        print("[INFO] Entered Phrase: {}\n".format(keyboard_input))
        if retVal != True:
            return None
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
                    print("[INFO] {} - Enter is pressed!\n")
                    return True, retString
                elif keyboard_input == "backspace":
                    retString = retString[:len(retString) - 1]
                else:
                    retString += keyboard_input
                    print(retString)
            else:
                print("[INFO] {} - Loop stopped since enter button not pressed!")
                return False, None
        except Empty:
            print("[INFO] {} - Timeout occured within {} seconds!\n"
                .format(self.__class__.__name__, self._elapse_time))
            return False, None

        # send request to start recording faces and wait until all faces are recorded
    def __start_recording_faces(self):
        retVal = False
        recording_response = None

        self.__publish_speech_message("custom", "Bitte Namen eintippen!")
        retVal, name = self.__evaluate_keyboard_inputs()
        if retVal == True:
            speech_str = 'Danke ' + name
            self.__publish_speech_message("custom", speech_str)
            self.__publish_speech_message("custom", "Ich starte mit der aufnahme von 10 Bildern!")
            self.__publish_speech_message("custom", "Bitte lachen")

            try:
                self._publish_led_ears(random = "on")
                request = rospy.ServiceProxy('/robofriend/facerecord', FaceRecordData)
                print("[INFO] {} - Sending request and Waiting for response!\n".format(__class__.__name__))

                for cnt in range(1, self.__pic_record + 1):     # to start from 1 up to self.__pic_record
                    recording_response = request(name, cnt)
                    if recording_response is not name:
                        name = recording_response               # in case when requested name already exists
                    try:
                        self.__publish_speech_message("custom", "{} Bild aufgenommen!".format(record_pic_speech[cnt]))
                    except KeyError:
                        self.__publish_speech_message("custom", "Weiteres Bild aufgenommen!")
                    sleep(1)
                print("[INFO] {} - Recording new faces finished!\n".format(__class__.__name__))
                retVal = True
                self.__publish_speech_message("custom", "Bin mit er aufnahme fertig")
            except rospy.ServiceException:
                print("[INFO] {} - Service call failed!".format(__class__.__name__))
                retVal = False
            finally:
                self.__publish_led_ears(random = "off")
                return retVal
        elif retVal == False:
            self.__publish_speech_message("custom", "Falschen Namen einegeben!")
            return False


    def __publish_speech_message(self, mode, text = None):
        self._msg_speech.mode = mode
        self._msg_speech.text = text
        self._pub_speech.publish(self._msg_speech)
        print("[INFO] {} - Speech Published Data: {}\n".format(self.__class__.__name__, self._msg_speech))

    def __publish_led_ears_message(self, random = "", repeat_num = [0, 0], rgb_color = []):
        self._msg_led_ears.random = random
        self._msg_led_ears.repeat_num = repeat_num
        self._msg_led_ears.rgb_color = rgb_color
        self._pub_led_ears.publish(self._msg_led_ears)
        print("[INFO] {} - Ears/Led Published Data: {}\n".format(self.__class__.__name__, self._msg_led_ears))

    def __publish_servo_cam_message(self, max_min = "", diff = 0):
        self._msg_servo_cam.max_min = max_min
        self._msg_servo_cam.diff = diff
        self._pub_servo_cam.publish(self._msg_servo_cam)
        print("[INFO] {} - Servo Camera Published Data: {}\n".format(self.__class__.__name__, self._msg_servo_cam))
