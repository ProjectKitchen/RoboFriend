from FaceDetectionNode.FaceDetectionDataHandler import *

# globals
runFlag = True

def node_stop():
    global runFlag
    runFlag = False
    print("[INFO] Stopping cam node!")

def node_start():
    print("[INFO] Ros FaceDetection Node started!\n!")

    facedetect = FaceDetectionDataHandler()
