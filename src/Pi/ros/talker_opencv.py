#!/usr/bin/env python3

import rospy
#from std_msgs.msg import String
import threading
#from pi_face_recognition_ros import *
import pi_face_recognition_ros
from facedetection_coordinates.msg import Coordinates

def talker():
    pub = rospy.Publisher('face_cordinates_topic', Coordinates, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)

    event_coordinates = threading.Event()
    event_ros = threading.Event()

    facerecog_thread = threading.Thread(
        name = "facedetect",
        target = pi_face_recognition_ros.face_recog,
        args = (event_coordinates, event_ros, ),
    )
    facerecog_thread.start()
    msg = Coordinates()

    """TODO: Using a mutex to protect global variable """

    while facerecog_thread.is_alive():
        if event_coordinates.wait():
            event_coordinates.wait()
            msg.y_top, msg.right, msg.bottom, msg.x_left,msg.face_name = pi_face_recognition_ros.coordinates
            event_coordinates.clear()
            print("[INFO] Send Data: {}".format(msg))
            pub.publish(msg)
            #rate.sleep()
        else:
            print("[INFO] Nothing to send!")
    print("[INFO] END ROS TALKER-NODE")



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
