from imutils.video import VideoStream
from imutils.video import FPS
import rospy
import threading
import os
import face_recognition
import imutils
import pickle
import cv2
import urllib
import numpy as np
import time
import os

# import ros message
from robofriend.msg import CamData
from robofriend.srv import SrvFaceRecordData, SrvFaceRecordDataResponse

class FaceDetectionDataHandler():

    def __init__(self):

        # publisher for detected faces
        self.__robobrain_pub = rospy.Publisher('T_CAM_DATA', CamData, queue_size = 20)
        self.__msg = CamData()
        self.__coordinates = []

        # declare service
        serv = rospy.Service('/robofriend/facerecord', SrvFaceRecordData, self.__service_handler)

        self.__face_record_event = threading.Event()

        # initialize the MJPG Stream Url to capture the frame
        self.__url = "http://localhost:8080/?action=stream"

        # set mjpg stream flag to a default value
        self.__mjpg_stream = False

        # configure Pi camera and load necessary files
        self.__face_recog_init()

        # create and start facerecognition thread
        self.__start_facedetect_thread()

    def __service_handler(self, request):
        self.__record_response = False
        self.__received_name = ""

        print("[INFO] {} - Request received: Name: {}\n".format(self.__class__.__name__, request.name))
        self.__received_name = request.name

        self.__set_event_face_record()

        #######################################
        #TODO: Start recording faces
        time.sleep(5)
        #######################################

        self.__clear_event_face_record()

        print("[INFO] {} - Recording finished!\n".format(self.__class__.__name__))
        self.__record_response = True
        return FaceRecordDataResponse(self.__record_response) # response that recording has finished

    def __face_recog_init(self):
        print("[INFO] {} - Loadings encodings and face detetcted")

        path = os.path.dirname(os.path.realpath(__file__))

        # location of haarcascade and encodings
        encodings_path = path + '/encodings.pickle'
        haarcascade_path = path + '/haarcascade_frontalface_default.xml'

        self.__detector = cv2.CascadeClassifier(haarcascade_path)
        try:
            encodings = open(encodings_path, "rb").read()
        except FileNotFoundError:
            print("[INFO] No pickle file found!\n")
            #TODO: in case of an empty pickle file

        # selecting the source of  the stream (either PiCamera or stream)
        vs = cv2.VideoCapture(self.__url)

        if vs.isOpened():
            print("[INFO] Pictures are captured via the mjpg-streamer")
            self._mjpg_stream = True
        else:
            print("[INFO] Pictures are captured directly by the pi-camera")
            vs = VideoStream(usePiCamera=True).start()

        time.sleep(2.0)

    def __start_facedetect_thread(self):
        thread = threading.Thread(
            target = self.__face_recognition
        )

        # set thread as a daemon
        thread.daemon = True

        # start thread
        thread.start()

    def __face_recognition(self):
        while True:
            if self.__is_face_record_running() is False:
                # grab the frame from the threaded video stream and resize it
                # to 500px (to speedup processing)
                if self._mjpg_stream == True:			# pictures are captured via the stream
                    vs = cv2.VideoCapture(self.__url)
                    stat, frame = vs.read()
                else:
                    frame = vs.read()

                #Flip camera vertically
                #frame = cv2.flip(frame, -1)
                frame = imutils.resize(frame, width=320, height=240)

                # convert the input frame from (1) BGR to grayscale (for face
                # detection) and (2) from BGR to RGB (for face recognition)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                # detect faces in the grayscale frame
                rects = self.__detector.detectMultiScale(gray, scaleFactor=1.2,
                    minNeighbors=5, minSize=(20, 20),
                    flags=cv2.CASCADE_SCALE_IMAGE)

                # OpenCV returns bounding box coordinates in (x, y, w, h) order
                # but we need them in (top, right, bottom, left) order, so we
                # need to do a bit of reordering
                boxes = [(y, x + w, y + h, x) for (x, y, w, h) in rects]

                # print("[INFO] Boxes: {}".format(boxes))

                if boxes != []:
                    # compute the facial embeddings for each face bounding box
                    encodings = face_recognition.face_encodings(rgb, boxes)
                    names = []

                    # loop over the facial embeddings
                    for encoding in encodings:
                        # attempt to match each face in the input image to our known
                        # encodings
                        matches = face_recognition.compare_faces(data["encodings"],
                            encoding)
                        name = "Unknown"

                        # check to see if we have found a match
                        if True in matches:
                            # find the indexes of all matched faces then initialize a
                            # dictionary to count the total number of times each face
                            # was matched
                            matchedIdxs = [i for (i, b) in enumerate(matches) if b]
                            counts = {}

                            # loop over the matched indexes and maintain a count for
                            # each recognized face face
                            for i in matchedIdxs:
                                name = data["names"][i]
                                counts[name] = counts.get(name, 0) + 1

                            # determine the recognized face with the largest number
                            # of votes (note: in the event of an unlikely tie Python
                            # will select first entry in the dictionary)
                            name = max(counts, key=counts.get)

                        # update the list of names
                        names.append(name)

                    # loop over the recognized faces
                    for ((top, right, bottom, left), name) in zip(boxes, names):
                        # draw the predicted face name on the image
                        cv2.rectangle(frame, (left, top), (right, bottom),
                            (0, 255, 0), 2)
                        y = top - 15 if top - 15 > 15 else top + 15
                        cv2.putText(frame, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.75, (0, 255, 0), 2)

                    self.__coordinates = list(boxes[0]).copy()
                    self.__coordinates.append(name)
                    print("[INFO] Coordinates in Submodule: {}".format(self.__coordinates))
                    msg.top, msg.right, msg.bottom, msg.left, msg.name = self.__coordinates
                    pub.publish(msg)

                time.sleep(0.2)

    def __set_event_face_record(self):
        self.__face_record_event.set()

    def __clear_event_face_record(self):
        self.__face_record_event.clear()

    def __is_face_record_running(self):
        return self.__face_record_event.is_set()