import rospy

# import ros message
from std_msgs.msg import Int8

# import ros modules
from ServoCamNode.ServoCamDataHandler import *

def node_start():
    print("[INFO] ROS Servo Camera Node started!\n")

    servocam = ServoCamDataHandler()
    rospy.Subscriber("T_SERVO_CAM_DATA", Int8, servocam.process_data)
