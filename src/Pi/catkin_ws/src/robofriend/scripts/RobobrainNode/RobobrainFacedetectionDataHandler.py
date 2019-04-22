from RobobrainStateHandler import *
from time import *
from threading import *
#from queue import *
from Queue import *
import rospy
import traceback

# import ros service
from robofriend.srv import SrvFaceRecordData
from robofriend.srv import SrvFaceDatabaseData
from robofriend.srv import SrvFaceHeartbeatData
from robofriend.srv import SrvFaceDrawData

# import ros messages
from robofriend.msg import SpeechData
from robofriend.msg import LedEarsData
from robofriend.msg import ServoCamData
from robofriend.msg import CamData

class RobobrainFacedetectionDataHandler():

    def __init__(self, queue):
        self._top = 0
        self._right = 0
        self._bottom = 0
        self._left = 0
        self._name = None
        self._keyboard_queue = queue
        self._face_familiarity = None

        # amount of recorded pictures
        self.__pic_record = 10
        self._elapse_time = 45
        self._service_elapse_time = 5

        self._facedetection_node_started = False
        self._facedetection_node_status_lock = Lock()

        self.__record_pic_speech = {
                                    1  : "Erstes", \
                                    2  : "Zweites", \
                                    3  : "Drittes", \
                                    4  : "Viertes", \
                                    5  : "Funftes", \
                                    6  : "Sechstes", \
                                    7  : "Siebentes", \
                                    8  : "Achtes", \
                                    9  : "Neuntes", \
                                    10 : "Zehntes"
                                    }

        # init publishers
        self._pub_speech = rospy.Publisher('/robofriend/speech_data', SpeechData, queue_size = 10)
        self._pub_led_ears = rospy.Publisher('/robofriend/led_ears_data', LedEarsData, queue_size = 10)
        self._pub_servo_cam = rospy.Publisher('/robofriend/servo_cam_data', ServoCamData, queue_size = 10)

        self._msg_speech = SpeechData()
        self._msg_led_ears = LedEarsData()
        self._msg_servo_cam = ServoCamData()
        self._search_new_face = Event()

        # init subscribers
        rospy.Subscriber("/robofriend/cam_data",  CamData, self._process_data)

        #init face service
        rospy.wait_for_service('/robofriend/face')
        self._face_request = rospy.ServiceProxy('/robofriend/face', SrvFaceDrawData)

        # init services
        try:
            rospy.wait_for_service('/robofriend/facerecord', timeout = self._service_elapse_time)
            rospy.wait_for_service('/robofriend/facedatabase', timeout = self._service_elapse_time)
            rospy.wait_for_service('/robofriend/fd_heartbeat', timeout = self._service_elapse_time)
        except rospy.ROSException:
            rospy.logwarn("{%s} - Facedetection node was not able to start within %s seconds therefore no facerecognition possible",
                self.__class__.__name__, str(self._elapse_time))
            self._face_detection_node_status(False)
        else:
            rospy.logdebug("{%s} - Facedection node started!", self.__class__.__name__)
            self._face_detection_node_status(True)


        thread = Thread(target = self._hb_thread)
        thread.daemon = True
        thread.start()

    def _face_detection_node_status(self, alive = False):
        if alive is True:
            self.__facerecord_request = rospy.ServiceProxy('/robofriend/facerecord', SrvFaceRecordData)
            self.__facedatabase_request = rospy.ServiceProxy('/robofriend/facedatabase', SrvFaceDatabaseData)
            self.__face_hb_request = rospy.ServiceProxy('/robofriend/fd_heartbeat', SrvFaceHeartbeatData)
            self._set_facedetection_status_flag(True)
        elif alive is False:
            self.__facerecord_request = None
            self.__facedatabase_request = None
            self.__face_hb_request = None
            self._set_facedetection_status_flag(False)

    def _hb_thread(self):
        rospy.logdebug("Heartbeat thread started!")
        while True:
            self.__face_hb_request = rospy.ServiceProxy('/robofriend/fd_heartbeat', SrvFaceHeartbeatData)
            try:
                ret = self.__face_hb_request(True)
            except rospy.ServiceException:
                rospy.logdebug("{%s} - Facedetection node not started yet!", rospy.get_caller_id())
                self._face_detection_node_status(False)
            else:
                rospy.logdebug("{%s} - Facedetection node started!", rospy.get_caller_id())
                self._face_detection_node_status(True)
            sleep(1)


    def _process_data(self, data):
        rospy.logdebug("{%s} - Received message: {%s}",
            self.__class__.__name__, str(data))
        if self.__is_facesearching_activated():
            self._top = data.top            # y top
            self._right = data.right        # x_w
            self._bottom = data.bottom      # y_h
            self._left = data.left          # x ------
            self._name = data.name
        else:
            pass

    def _start_facedetection(self):
        self._face_familiarity = None

        if self._get_facedetection_status_flag() is False:
            rospy.logwarn("{%s} - Facedection node not started therefore leave face interaction state", self.__class__.__name__)
            face_node = False
            self._face_familiarity = None
            self.__publish_speech_message("custom", "Kann derzeit keine Gesichter detektieren!")
        elif self._get_facedetection_status_flag() is True:
            cnt = 0
            while cnt < 2:
                self.__publish_led_ears_message(rgb_color = [0, 0, 15])
                cnt += 1
                sleep(1.1)
            face_node = True
            face_detectded, face_grade = self.__face_search()
            if  face_detectded is True:
                rospy.logdebug("{%s} - Face detected! Further steps are activated!\n", self.__class__.__name__)
                self.__publish_speech_message("custom", "Ich habe jemanden gefunden")
                self._center_face()
                self.__stop_searching_new_face()
                face_grade.lower()
                if face_grade != "unknown":
                    self.__known_face_speech(face_grade)
                    if self.__take_picture_known_face(face_grade) is True:
                        self.__create_database()
                    else:
                        self.__publish_speech_message("custom", "Okey dann nehme ich keine weiteren Bilder von dir auf")
                    self._face_familiarity = face_grade       # in case of a known face
                elif face_grade == "unknown":
                    self.__unknown_face_speech()
                    if self.__yes_no_keyboard_request() is True:
                        rospy.logdebug("{%s} - Start recording Pictures!\n", self.__class__.__name__)
                        retVal, name = self.__start_recording_faces()
                        if retVal is True:    # start recording faces
                            rospy.logdebug("{%s} - Faces are recorded!\n", self.__class__.__name__)
                            self.__create_database()
                        elif retVal is False:
                            rospy.logwarn("{%s} - Recording new faces failed!\n", self.__class__.__name__)
                            self._face_familiarity = "unknown"
                    else:
                        self.__publish_speech_message("custom", "Ich darf mit fremden Leuten nicht reden")
                        rospy.logdebug("{%s} - Recording Pictures not allowed! Start searching new face\n", self.__class__.__name__)
            elif face_detectded is False:
                self.__publish_speech_message("custom", "Keine menschensseele hier!")
                rospy.logdebug("{%s} -  No Face detected within {%s} seconds! Change State to IDLE State\n", self.__class__.__name__, str(self._elapse_time))
                self._face_familiarity = "no_person"

            rospy.logdebug("Facenode: %s, Face Familiarity: %s", face_node, self._face_familiarity)
            self._set_eyes_to_default()
            return self._face_familiarity

    def __face_search(self):
        self.__publish_speech_message("custom", "Schauen wir mal ob ich bekannte Gesichter finde!")
        self.__start_searchig_new_face()
        self.__publish_servo_cam_message("min")
        start_time = self.__time_request()
        while self.__time_request() - start_time < self._elapse_time:
            self.__publish_servo_cam_message(diff = 10)
            if self._name is not None:
                return True, self._name
            sleep(2)
        else:
            rospy.logdebug("{%s} - Time for detecting a face expired! Leave Facedetection interaction state!", self.__class__.__name__)
            return False, None

    def _center_face(self):
        cnt = 0
        while cnt < 5:
            rospy.logwarn("Coordinates - y_top: %s, x_w: %s, y_h: %s, x: %s", self._top, self._right, self._bottom, self._left)
            if self._left < 90:
                # move left
                rospy.logwarn("{%s} - left: %s, therefore move left!",
                    rospy.get_caller_id(), str(self._left))
                if 0 <= self._left <= 45:
                    self._face_request('set_eyes', [95, -7])
                elif 46 <=  self._left <= 75:
                    self._face_request('set_eyes', [50, -7])
                else:
                    self._face_request('set_eyes', [20, -7])
            elif self._left > 150:
                # move right
                rospy.logwarn("{%s} - left: %s, therefore move right!",
                    rospy.get_caller_id(), str(self._left))
                if 150 <= self._left <= 180:
                    self._face_request('set_eyes', [-20, -7])
                elif 181 <= self._left <= 200:
                    self._face_request('set_eyes', [-50, -7])
                else:
                    self._face_request('set_eyes', [-95, -7])
            else:
                rospy.logwarn("{%s} - Face in the center!",
                    rospy.get_caller_id())
                self._set_eyes_to_default()
            sleep(2)
            cnt += 1

    def __time_request(self):
        return time()

    def _set_eyes_to_default(self):
        self._face_request('set_eyes', [-7, -7])

    def __start_searchig_new_face(self):
        rospy.logdebug("{%s} - Set event to start searching new faces!", self.__class__.__name__)
        self._search_new_face.set()

    def __stop_searching_new_face(self):
        rospy.logdebug("{%s} - Clear event to stop searching for new faces!", self.__class__.__name__)
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
                rospy.logdebug("{%s} - %s new pictures are recorded!\n",
                    self.__class__.__name__, str(cnt))
                return True
        else:
            rospy.logdebug("{%s} - Person rejected to record new pictures!\n",
                self.__class__.__name__)
            return False

    def __yes_no_keyboard_request(self):
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
                self.__publish_speech_message("custom", "Du hast nicht enter getippt!")
                return False, None
        except Empty:
            rospy.logdebug("{%s} - Timeout occured within {%s} seconds!\n"
                , self.__class__.__name__, self._elapse_time)
            self.__publish_speech_message("custom", "Du warst mit der eingabe zu langsam")
            return False, None

        # send request to start recording faces and wait until all faces are recorded
    def __start_recording_faces(self):
        retVal = False
        recording_response = None
        name_lastname = None
        name = None

        self.__publish_speech_message("custom", "Bitte Namen eintippen!")
        retVal, name = self.__evaluate_keyboard_inputs()
        if retVal is True:
            name = name.lower()
            check_database = self.__check_name_database(name)
            if check_database is None:          # in case of an error
                rospy.logwarn("{%s} - Error while checking name in database!",
                    self.__class__.__name__)
                retVal = False
                return retVal, name

            if check_database is False:       # no directory available with this name => therefore create new directory
                if self.__take_picture_new_face(name) is False:
                    retVal = False
                    return retVal, name
                else:
                    rospy.logdebug("{%s} - Recording new {%s} faces finished!",
                        self.__class__.__name__, str(self.__pic_record))
                    retVal = True
                    return retVal, name
            elif check_database is True:   # a directory with the entered name already exists
                self.__publish_speech_message("custom", "Ich kenne bereits eine Person mit diesem Namen!")
                self.__publish_speech_message("custom", "Bitte gebe deinen Namen mit deinem Nachnamen ein!")

                while name_lastname is None or name_lastname == name:
                    retVal, name_lastname = self.__evaluate_keyboard_inputs()
                    if name_lastname == name:
                        self.__publish_speech_message("custom", "Bitte unterschiedlichen Namen eingeben!")
                else:
                    rospy.logdebug("{%s} - Different name entered!", self.__class__.__name__)
                    if self.__take_picture_new_face(name_lastname) is False:
                        retVal = False
                        return retVal, name
                    else:
                        rospy.logdebug("{%s} - Recording new {%s} faces finished!",
                            self.__class__.__name__, str(self.__pic_record))
                        retVal = True
                        return retVal, name
        elif retVal is False:
            rospy.logdebug("{%s} - No name entered", self.__class__.__name__)
            self.__publish_speech_message("custom", "Falschen Namen einegeben!")
            return retVal, name

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
        rospy.logdebug("{%s} - Request for recording new picture!\n",
            self.__class__.__name__)

        self.__publish_led_ears_message(rgb_color = [15, 0, 0]) # to flush the ears in red
        self.__publish_speech_message("custom", "Enter tippen wenn das Foto aufgenommen werden soll")
        while enter is not "":
            retVal, enter = self.__evaluate_keyboard_inputs()
            rospy.logdebug("{%s} - Entered Input: {%s}\n",
                self.__class__.__name__, str(enter))
            if retVal is False:
                self.__publish_speech_message("custom", "Bitte enter eingeben")
            elif enter is not "":
                self.__publish_speech_message("custom", "Bitte nur enter eingeben")
        else:
            rospy.logdebug("{%s} - Sending request and waiting for response!\n",
                self.__class__.__name__)
            recording_response = self.__facerecord_request(check_name = False, name = name)
            if recording_response.picture_taken is True:
                self.__publish_led_ears_message(rgb_color = [0, 15, 0])
                rospy.logdebug("{%s} - New Picture reorded!", self.__class__.__name__)
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
            rospy.logdebug("{%s} - No directory with that name!\n", self.__class__.__name__)
            return False                             # return False if no directory with this name exists
        elif response.check_name_resp is True:
            rospy.logdebug("{%s} - Directory with this name in database!\n", self.__class__.__name__)
            return True                             # return True if a directory with this name already exists

    def __create_database(self):
        retVal = False

        self.__publish_speech_message("custom", "Ich versuche dein Gesicht und dein Namen zu merken")
        self.__publish_speech_message("custom", "Das kann etwas dauern")
        try:
            self.__publish_led_ears_message(random = "on")
            response = self.__facedatabase_request(True)
            if response.response == True:
                rospy.logdebug("{%s} - Creating database is finished!", self.__class__.__name__)
                self.__publish_speech_message("custom", "Bin mit dem merken fertig!")
                self.__publish_speech_message("custom", "Willkommen im meinem Freundeskreis")
                retVal = True
        except rospy.ServiceException:
            rospy.logdebug("{%s} - Service call failed!", self.__class__.__name__)
            retVal = False
        except Exception :
            traceback.print_exc()
        finally:
            self.__publish_led_ears_message(random = "off")
            if retVal is True:
                rospy.logdebug("{%s} - New Database created!\n", self.__class__.__name__)
            return retVal

    @property
    def _face_familiarity(self):
        return self._face_familiarity

    def __publish_speech_message(self, mode, text = ""):
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

    def _set_facedetection_status_flag(self, alive = False):
        self._facedetection_node_status_lock_aquire()
        self._facedetection_node_started = alive
        self._facedetection_node_status_lock_release()

    def _get_facedetection_status_flag(self):
        self._facedetection_node_status_lock_aquire()
        retVal = None
        retVal = self._facedetection_node_started
        self._facedetection_node_status_lock_release()
        return retVal

    def _facedetection_node_status_lock_aquire(self):
        self._facedetection_node_status_lock.acquire()

    def _facedetection_node_status_lock_release(self):
        self._facedetection_node_status_lock.release()
