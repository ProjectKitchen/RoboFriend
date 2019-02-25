from FaceDetectionNode.FaceDetectionDataHandler import *
#import rospy
#from FaceDetectionDataHandler import *


# globals
runFlag = True

#def node_stop():
#    global runFlag
#    runFlag = False
#    print("[INFO] Stopping cam node!")

def main():
    print("[INFO] Ros FaceDetection Node started!\n!")

#    rospy.init_node('FaceDetection_node', anonymous = True)
    facedetect = FaceDetectionDataHandler()

def node_start():
    print("[INFO] Ros FaceDetection Node started!\n!")

    #rospy.init_node('FaceDetection_node', anonymous = True)
    facedetect = FaceDetectionDataHandler()


if __name__ == "__main__":
    main()
