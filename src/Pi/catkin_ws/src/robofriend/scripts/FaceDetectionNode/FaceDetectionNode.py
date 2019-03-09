#!/usr/bin/env python3

from FaceDetectionDataHandler import *
import rospy

def shutdown():
    rospy.signal_shutdown("Stopping Face Detecion node!")

def FaceDetecion():
    rospy.init_node("robofriend_face_detection_node", log_level = rospy.INFO)
    rospy.loginfo("Starting Face Detecion Node!")

    fd = FaceDetectionDataHandler()

    rate = rospy.Rate(2) # 2 fps

    while not rospy.is_shutdown():
        fd._face_recognition()

        rate.sleep()

if __name__ == '__main__':
    try:
        FaceDetecion()
    except rospy.ROSInterruptException:
        pass
