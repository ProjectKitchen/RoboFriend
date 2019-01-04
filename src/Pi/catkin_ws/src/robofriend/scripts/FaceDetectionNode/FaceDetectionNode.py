from ros_robofriend.msg import CamData
from imutils.video import VideoStream
from imutils.video import FPS
import face_recognition
import argparse
import imutils
import pickle
import cv2
import urllib
import numpy as np
import time
import os
import rospy
import threading


# globals
runFlag = True

def shutdown():
    rospy.signal_shutdown("Stopping Face Detection node!")

def node_start():
    print("[INFO] Ros Cam Node start!")
    coordinates = 0

    pub = rospy.Publisher('/robofriend/cam_data', CamData, queue_size = 20)

    msg = CamData()

    # include message queue
    face_recog_thread = threading.Thread(
        target = face_recog,
        args = (pub, msg, )
    )
    
    # set thread as a daemon
    face_recog_thread.daemon = True

    # start the face_recog thread
    face_recog_thread.start()

def face_recog(pub, msg):
    global runFlag
    coordinates = []

    path = os.path.dirname(os.path.realpath(__file__))

    #print(talker_opencv.var_coordinates)

    # load the known faces and embeddings along with OpenCV's Haar
    # cascade for face detection
    print("[INFO] loading encodings + face detector...")

    # path of the encodings and the haarcascade file
    encodings_path = path + "/encodings.pickle"
    haarcascade_path = path + "/haarcascade_frontalface_default.xml"


    #detector = cv2.CascadeClassifier("harcascade_frontalface_default.xml")
    #encodings = open("encodings.pickle", "rb").read()
    detector = cv2.CascadeClassifier(haarcascade_path)
    encodings = open(encodings_path, "rb").read()
    data = pickle.loads(encodings)

    # initialize the MJPG Stream Url to capture the frame
    urls = "http://localhost:8080/?action=stream"

    # set the mjpg stream flag to a default value
    mjpg_stream = False

    # initialize the video stream and allow the camera sensor to warm up
    print("[INFO] starting video stream...")

    # selecting the source of  the stream (either PiCamera or stream)
    vs = cv2.VideoCapture(urls)
    #vs = VideoStream(usePiCamera=True).start()

    if vs.isOpened():
        print("[INFO] Pictures are captured via the mjpg-streamer")
        mjpg_stream = True
    else:
        print("[INFO] Pictures are captured directly by the pi-camera")
        vs = VideoStream(usePiCamera=True).start()

    time.sleep(2.0)

    # loop over frames from the video file stream
    while runFlag:
        # grab the frame from the threaded video stream and resize it
        # to 500px (to speedup processing)
        if mjpg_stream == True:			# pictures are captured via the stream
            vs = cv2.VideoCapture(urls)
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
        rects = detector.detectMultiScale(gray, scaleFactor=1.2,
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

            coordinates = list(boxes[0]).copy()
            coordinates.append(name)
            print("[INFO] Coordinates in Submodule: {}".format(coordinates))
            msg.top, msg.right, msg.bottom, msg.left, msg.name = coordinates
            pub.publish(msg)

        # display the image to our screen
        #v2.imshow("Frame", frame)
        #key = cv2.waitKey(1) & 0xFF

        # if the `q` key was pressed, break from the loop
        # if key == ord("q"):
        #     break

        time.sleep(0.2)

    print("[INFO] END FACE RECOGNITION THREAD")
    # do a bit of cleanup
    #cv2.destroyAllWindows()
    #cv2.destroyAllWindows()

class FaceDetectionDataHandler(object):
    def __init__(self, pub):
        self._pub = pub

    def publishMsg(self):
        # TODO: do something cool here
        pass

def FaceDetection():
    rospy.loginfo("Starting Face Detection node!")
    rospy.init_node('robofriend/cam_data', anonymous = True)

    pub = rospy.Publisher('/robofriend/cam_data', CamData, queue_size = 20)

    dh = FaceDetectionDataHandler(pub)

    rate = rospy.Rate(1) # 1hz

    while not rospy.is_shutdown():
        # TODO: implement methode
        dh.publishMsg() 
        rate.sleep() # make sure the publish rate maintains at the needed frequency

if __name__ == '__main__':
    try:
        FaceDetection()
    except rospy.ROSInterruptException:
        pass