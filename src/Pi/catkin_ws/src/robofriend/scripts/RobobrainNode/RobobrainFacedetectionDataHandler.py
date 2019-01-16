from RobobrainNode.RobobrainStateHandler import *
from threading import *
from time import *
from queue import *
import rospy

# import ros service
#from ros_robofriend.srv import FaceRecord
#TODO: create service

class RobobrainFacedetectionDataHandler():

    def __init__(self, robostate, publisher_handler, queue):
        self._top = 0
        self._right = 0
        self._bottom = 0
        self._left = 0
        self._name = None

        self._robostate = robostate
        self._pub = publisher_handler
        self._keyboard_queue = queue

        self.__start_thread()

        #self._actual_state = self._facedetection_states['IDLE']
        self._elapse_time = 30
        self._search_new_face = Event()

        rospy.wait_for_service('/robofriend/start_facerecord')

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
            while self._robostate.state == RobobrainStateHandler.robostate["FACEDETECTION"]:
                face_detectded, face_grade = self.__face_search()
                self.__stop_searching_new_face()
                if  face_detectded is True:
                    print("Face detected! Further steps are activated!\n")
                    self._pub.speech_message_publish("custom", "Ich habe jemanden gefunden")
                    if face_grade != "unknown":
                        self.__known_face_speech(face_grade)
                        #TODO: Do something in case of a known face
                    elif face_grade == "unknown":
                        self.__unknown_face_speech()
                        if self.__yes_no_keyboard_request() is True:
                            print("[INFO] {} - Start recording Pictures!\n".format(self.__class__.__name__))
                            self._pub.speech_message_publish("custom", "Ich starte mit der aufnahme von 10 Bildern!")
                            self._pub.speech_message_publish("custom", "Bitte lachen")
                            #TODO: Start recording pictures
                            if self.__start_recording_faces() is True:
                                print("[INFO] {} - Pictures are recorded!\n".format(self.__class__.__name__))

                        else:
                            print("[INFO] {} - Recording Pictures not allowed! Start searching new face\n".format(self.__class__.__name__))
                            break
                elif face_detectded is False:
                    # self.__stop_searching_new_face()
                    self._pub.speech_message_publish("custom", "Keine menschensseele hier!")
                    print("[INFO] No Face detected within {} seconds! Change State to IDLE State\n".format(self._elapse_time))
                    self._robostate.state = RobobrainStateHandler.robostate["IDLE"]
                    break
            else:
                sleep(1)

    def __face_search(self):
        self._pub.speech_message_publish("custom", "Ich schaue mich dann mal nach Menschen um!")
        self.__start_searchig_new_face()
        self._pub.servo_cam_message_publish("min")
        start_time = self.__time_request()
        while self.__time_request() - start_time < self._elapse_time:
            self._pub.servo_cam_message_publish(diff = 10)
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
        self._pub.ears_led_message_publish(rgb_color = [0, 15, 0]) # to flush the ears in green
        self._pub.speech_message_publish("custom", "Ich kenne dich!")
        self._pub.speech_message_publish("custom", "Du bist " + name)

    def __unknown_face_speech(self):
        self._pub.ears_led_message_publish(rgb_color = [15, 0, 0]) # to flush the ears in red
        self._pub.speech_message_publish("custom", "Ich kenne dich nicht!")
        self._pub.speech_message_publish("custom", "Ich darf mit fremden Leuten nicht reden")
        self._pub.speech_message_publish("custom", "Darf ich Bilder von dir aufnehmen?")

    def __yes_no_keyboard_request(self):
        #TODO: replace with voice detection
        print("[INFO] Waiting for yes or no!\n")
        retVal, keyboard_input = self.__evaluate_keyboard_inputs()
        print("[INFO] Entered Phrase: {}\n".format(keyboard_input))
        if retVal != True:
            return None
        else:
            keyboard_input.lower()
            if keyboard_input == "ja":
                return True
            elif keyboard_input == "nein":
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
            # rospy.wait_for_service('/robofriend/get_record_status')
            try:
                self.__pub.ears_led_message_publish(random = "on")
                request = rospy.ServiceProxy('/robofriend/start_facerecord', FaceRecord)
                print("[INFO] {} - Sending request and Waiting for response!\n".format(__class__.__name__))
                recording_response = request(True)
                print("[INFO] {} - Recording new faces finished!\n".format(__class__.__name__))
                retVal = True
            except rospy.ServiceException:
                print("[INFO] {} - Service call failed!".format(__class__.__name__))
                retVal = False
            finally:
                self.__pub.ears_led_message_publish(random = "off")
                return retVal
