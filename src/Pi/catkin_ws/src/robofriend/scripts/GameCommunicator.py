#!/usr/bin/env python
import rospy

def shutdown():
    rospy.signal_shutdown("Stopping Game Communicator node!")

def GameCommunicator():
    rospy.init_node("robofriend_game_communicator", log_level = rospy.INFO)
    rospy.loginfo("Starting Game Communicator  Handler node!")
    
    rospy.spin()

if __name__ == '__main__':
    try:
        GameCommunicator()
    except rospy.ROSInterruptException:
        pass