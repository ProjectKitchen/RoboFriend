import rospy

# import ros message
from robofriend.msg import ServoCamData

# import ros modules
from ServoCamNode.ServoCamDataHandler import *

def node_start():
    print("[INFO] ROS Servo Camera Node started!\n")

    servocam = ServoCamDataHandler()
    rospy.Subscriber("/robofriend/servo_cam_data", ServoCamData, servocam.process_data)
