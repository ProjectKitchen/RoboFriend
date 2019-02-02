from imutils.video import VideoStream
from imutils.video import FPS
from imutils import paths
from imutils import *
import rospy
import threading
import os
import face_recognition
import pickle
import cv2
import urllib
import numpy as np
import time
import os
import random

# import ros message
from robofriend.msg import CamData
from robofriend.srv import SrvFaceRecordData, SrvFaceRecordDataResponse
from robofriend.srv import SrvFaceDatabaseData, SrvFaceDatabaseDataResponse


class FaceDetectionDataHandler():

    def __init__(self):

        # publisher for detected faces
        self.__robobrain_pub = rospy.Publisher('/robofriend/cam_data', CamData, queue_size = 20)
        self.__msg = CamData()
        self.__coordinates = []

        # declare face record service
        serv = rospy.Service('/robofriend/facerecord', SrvFaceRecordData, self.__face_record_service_handler)

        # declare creat database service
        serv = rospy.Service('/robofriend/facedatabase', SrvFaceDatabaseData, self.__face_create_database_service_handler)

        self.__face_recognition_event = threading.Event()

        # initialize the MJPG Stream Url to capture the frame
        self.__vs = None
        self.__url = "http://localhost:8080/?action=stream"

        # set mjpg stream flag to a default value
        self.__mjpg_stream = False

        # configure Pi camera and load necessary files
        self.__face_recog_init()

        # create and start facerecognition thread
        self.__start_facedetect_thread()

    def __face_recog_init(self):
        print("[INFO] {} - Loadings encodings and face detetcted")

        self.__path = os.path.dirname(os.path.realpath(__file__))

        # location of haarcascade and encodings
        encodings_path = self.__path + '/encodings.pickle'
        haarcascade_path = self.__path + '/haarcascade_frontalface_default.xml'

        self.__detector = cv2.CascadeClassifier(haarcascade_path)
        self.__data = pickle.loads(open(encodings_path, "rb").read())

        # selecting the source of  the stream (either PiCamera or stream)
        self.__vs = cv2.VideoCapture(self.__url)

        if self.__vs.isOpened():
            print("[INFO] Pictures are captured via the mjpg-streamer")
            self.__mjpg_stream = True
        else:
            try:
                self.__vs = VideoStream(usePiCamera = True).start()
                print("[INFO] Pictures are captured directly by the pi-camera")
            except ImportError:
                print("[INFO] Pictures are captured from the webcam")
                self.__vs = VideoStream(src = 0).start()

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
            if self.____is_face_recognition_blocked() is False:
                # grab the frame from the threaded video stream and resize it
                # to 500px (to speedup processing)
                if self.__mjpg_stream == True:			# pictures are captured via the stream
                    self.__vs = cv2.VideoCapture(self.__url)
                    stat, frame = vs.read()
                else:
                    frame = self.__vs.read()

                #Flip camera vertically
                #frame = cv2.flip(frame, -1)
                frame = resize(frame, width=320, height=240)

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
                        matches = face_recognition.compare_faces(self.__data["encodings"],
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
                                name = self.__data["names"][i]
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
                    #print("[INFO] Coordinates in Submodule: {}".format(self.__coordinates))
                    self.__msg.top, self.__msg.right, self.__msg.bottom, self.__msg.left, self.__msg.name = self.__coordinates
                    self.__robobrain_pub.publish(self.__msg)

                time.sleep(0.2)

    def __face_record_service_handler(self, request):
        retVal = False
        num = 0

        rospy.loginfo("{%s} - Face record request received: {%s} {%s}\n",
            self.__class__.__name__, str(request.check_name), str(request.name))

        if request.check_name is True:
            check_name = self.__check_name_database(request.name)
            if check_name is True:
                return SrvFaceRecordDataResponse(check_name_resp = True, picture_taken = False)
            elif check_name is False:
                return SrvFaceRecordDataResponse(check_name_resp = False, picture_taken = False)
        elif request.check_name is False:
            rospy.loginfo("{%s} - Starting with recording of new faces\n",
                self.__class__.__name__)
            check_name = self.__check_name_database(request.name)
            if check_name is True:
                num = self.__count_images(request.name)
            elif check_name is False:
                num = self.__create_dir(request.name)

            self.__set_event_block_face_recognition()
            if self.__record_picture(request.name, num) is True:
                retVal = True
            else:
                retVal = False
            self.__clear_event_block_face_recognition()

            return SrvFaceRecordDataResponse(check_name_resp = True, picture_taken = retVal)

    def __check_name_database(self, name):
        path = self.__path + "/dataset"
        if name in os.listdir(path):
            return True         # when directory with this name already exists
        else:
            return False        # when no directory with this name exists

    def __count_images(self, name):
        path = self.__path + "/dataset" + "/" + name
        return len(list(paths.list_images(path)))

    def __create_dir(self, name):
        path = self.__path + "/dataset" + "/" + name
        os.makedirs(path)
        rospy.loginfo("{%s} - New directory with the name %s created",
            self.__class__.__name__, name)
        return 0

    def __record_picture(self, name, num):
        retVal = False
        path = self.__path + "/dataset" + "/" + name

        frame = self.__vs.read()
        p = os.path.sep.join([path, "{}.png".format(
            str(num).zfill(5))])

        write_status = cv2.imwrite(p, frame)
        if write_status is True:
            rospy.loginfo("{%s} - Picture %s is recorded\n",
                self.__class__.__name__, num)
            retVal = True
        elif write_status is False:
            rospy.logwerr("{%s} - Picture could not be recorded!\n",
                self.__class__.__name__)
            retVal = False
        return retVal

    def __face_create_database_service_handler(self, request):
        print("[INFO] {} - Creating Database Request received: Name: {}\n".
            format(self.__class__.__name__, request.create_database))

        self.__set_event_block_face_recognition()


        self.__create_database()
        time.sleep(10)

        self.__clear_event_block_face_recognition()

        print("[INFO] {} - Creating Database finished!\n".format(self.__class__.__name__))
        return SrvFaceDatabaseDataResponse(True) # response that recording has finished

    def __create_database(self):
        dataset_path = self.__path + '/dataset'
        knownEncodings = []
        knownNames = []

        image_paths = list(paths.list_images(dataset_path))
        for (i, image_path) in enumerate (image_paths):
            print("[INFO] processing image {}/{}".format(i + 1,
                len(image_paths)))
            name = image_path.split(os.path.sep)[-2]

            image = cv2.imread(image_path)
            rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            boxes = face_recognition.face_locations(rgb, model = "hog")

            encodings = face_recognition.face_encodings(rgb, boxes)

            for encoding in encodings:
                knownEncodings.append(encoding)
                knownNames.append(name)

        data = {"encodings": knownEncodings, "names": knownNames}
        f = open("encodings.pickle", "wb+")
        f.write(pickle.dumps(data))
        f.close()

    def __set_event_block_face_recognition(self):
        self.__face_recognition_event.set()

    def __clear_event_block_face_recognition(self):
        self.__face_recognition_event.clear()

    def ____is_face_recognition_blocked(self):
        return self.__face_recognition_event.is_set()
