from FaceDetectionNode import *
from ObjectDetection import *
import rospy

RUNFLAG = True

def stop():
    global RUNFLAG

    rospy.logwarn("### shutting down! ###\n\n")
    RUNFLAG = False

def handler_stop_signals(sigum, frame):
    stop()

def main():
    global RUNFLAG

    rospy.loginfo("Starting Computer Vision Nodes!\n")

    rospy.init_node("ComputerVision_Node", anonymous = True)

    rospy.logdebug("Start ROS Face DetectionNode!\n")
    FaceDetectionNode.node_start()
    rospy.logdebug("Start ROS Object Detection Node!\n")
    ObjectDetectionNode.node_start()

    # setting up signal handler for Shutdown
    signal.signal(signal.SIGINT, handler_stop_signals)
    signal.signal(signal.SIGTERM, handler_stop_signals)
    rospy.loginfo("*** startup completed! ***\n\n")

    while RUNFLAG:
        time.sleep(0.5)

if __name__ == "__main__":
    main()
