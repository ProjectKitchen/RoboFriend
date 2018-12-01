#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def path_data_c(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.data)
    
def PathPlanner():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'pathplanner' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('pathplanner', anonymous=True)

    rospy.Subscriber("T_PATH_DATA", String, path_data_c)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    PathPlanner()