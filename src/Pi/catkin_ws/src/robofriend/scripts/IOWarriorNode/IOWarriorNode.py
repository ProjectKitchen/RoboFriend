import rospy

# import ros message
from robofriend.msg import IOWarriorData

# import ros modules
from IOWarriorNode.IOWarriorDataHandler import *

def node_start():
    print("[INFO] ROS IOWarrior Node started!\n")

    iowarrior = IOWarriorDataHandler()
    rospy.Subscriber('T_IOWARRIOR_DATA', IOWarriorData, iowarrior.process_data)
