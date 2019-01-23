import rospy

# import ros message
from ros_robofriend.msg import ServoCamData

# import ros modules
from ServoCamNode.ServoCamDataHandler import *

def node_start():
    print("[INFO] ROS Servo Camera Node started!\n")

    servocam = ServoCamDataHandler()
    rospy.Subscriber("T_SERVO_CAM_DATA", ServoCamData, servocam.process_data)
