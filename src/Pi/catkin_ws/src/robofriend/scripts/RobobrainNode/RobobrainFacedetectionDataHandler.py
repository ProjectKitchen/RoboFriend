from RobobrainNode.RobobrainStateHandler import *
from threading import *
from time import *

class RobobrainFacedetectionDataHandler():

    def __init__(self, robostate, publisher_handler):
        self._top = 0
        self._right = 0
        self._bottom = 0
        self._left = 0
        self._name = None

        self._robostate = robostate
        self._pub = publisher_handler

        self.__start_thread()

        self._facedetection_states = {
            'FACE_SEARCH' : 1, \
            'KNOWN_FACE' : 2, \
            'UNKNOWN_FACE' : 3, \
            'IDLE': 4
        }
        self._actual_state = self._facedetection_states['IDLE']
        self._search_time = 30
        self._search_new_face = Event()

    def process_data(self, data):
        print("[INFO] {} - Received message: {} ".format(self.__class__.__name__, data))
        if self._actual_state == self._facedetection_states["FACE_SEARCH"] and self.__is_facesearching_activated():
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
                print("{} - Within Facedetection state - change interal state to FACE_SEARCH\n".format(self.__class__.__name__))
                self._actual_state = self._facedetection_states['FACE_SEARCH']
                if self._actual_state == self._facedetection_states["FACE_SEARCH"]:
                    retVal, face_grade = self.__face_search()
                    self.__stop_searching_new_face()
                    if  retVal is True:
                        print("Face detected! Further steps are activated!")
                        self._pub.speech_message_publish("custom", "Ich habe jemanden gefunden")
                        if face_grade != "unknown":
                            self._pub.ears_led_message_publish(rgb_color = [0, 15, 0]) # to flush the ears in green
                            self._pub.speech_message_publish("custom", "Ich kenne dich!")
                            self._pub.speech_message_publish("custom", "Du bist " + face_grade)
                        else:
                            self._pub.ears_led_message_publish(rgb_color = [15, 0, 0]) # to flush the ears in red
                            self._pub.speech_message_publish("custom", "Ich kenne dich nicht!")
                            self._pub.speech_message_publish("custom", "Meine Robomama hat gesagt, ich solle nicht mit fremden menschen reden!")
                    else:
                        self._pub.speech_message_publish("custom", "Keine menschliche Spezies hier!")
                        print("No Face detected within {} seconds! Change State to IDLE State\n".format(self._search_time))
                        self._robostate.state = RobobrainStateHandler.robostate["IDLE"]
                        break
            else:
                sleep(1)
                # print("State changed! Facedetection Thread is stoppped\n")

    def __face_search(self):
        self._pub.speech_message_publish("custom", "Ich shaue mich dann mal nach Menschen um!")
        self.__start_searchig_new_face()
        self._pub.servo_cam_message_publish("min")
        start_time = self.__time_request()
        while self.__time_request() - start_time < self._search_time:
            self._pub.servo_cam_message_publish(diff = 10)
            if self._name is not None:
                return True, self._name
            sleep(4)
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
    #top
    @property
    def top(self):
        return self._top

    @top.setter
    def top(self, value):
        self._top = value

    #right
    @property
    def right(self):
        return self._right

    @right.setter
    def right(self, value):
        self._right = value

    #left
    @property
    def left(arg):
        return self._left

    @left.setter
    def left(self, value):
        self._left = value

    #bottom
    @property
    def bottom(self):
        return self._bottom

    @bottom.setter
    def bottom(self, value):
        self._bottom = value

    #name
    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        self._name = value
