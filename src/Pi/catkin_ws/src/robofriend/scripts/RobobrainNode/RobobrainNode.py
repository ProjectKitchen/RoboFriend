#!/usr/bin/env python
import os, sys, rospy
import threading
# import queue

# from Queue import *
import Queue

# import ROS messages
from robofriend.msg import PCBSensorData
from robofriend.msg import VoiceData

# import ROS modules
from RobobrainFacedetectionDataHandler import *
from RobobrainKeyboardDataHandler import *
from RobobrainPCBSensorDataHandler import *
from RobobrainVoicedetectionDataHandler import *
from RobobrainObjectdetectionHandler import *
from RobobrainStateHandler import *

def shutdown():
    rospy.signal_shutdown("Stopping Robobrain node!")

def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'RoboBrain' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('robofriend_robobrain', anonymous = True, log_level = rospy.INFO)
    rospy.loginfo("Starting Robobrain node!")

    # publish here
    # pub = rospy.Publisher('topic_name', MsgType, queue_size = 10)

    event = threading.Event()

    # queue to pass messages from keyboardDataHandler to other Handlers
    kb_queue = Queue.Queue()

    # voice queue to coomunicate from RobobrainVoice to RobobrainFaceDetection
    # voice_queue = Queue.Queue()

    fd = RobobrainFacedetectionDataHandler(kb_queue)
    vd = RobobrainVoicedetectionDataHandler(kb_queue)
    obj = RobobrainObjectdetectionHandler()

    # sets actual state to IDLE and starts thread
    statehandler = RobobrainStateHandler(event, fd, vd, obj, kb_queue)

    sensors = RobobrainPCBSensorDataHandler(statehandler)
    # odo = RobobrainOdometryDataHandler()
    # ir  = RobobrainInfraredDataHandler()
    kb = RobobrainKeyboardDataHandler(statehandler, event, kb_queue)

    rospy.Subscriber("/robofriend/pcb_sensor_data", PCBSensorData, sensors.process_data)
    # rospy.Subscriber("/robofriend/odom_data", Pose, odo.process_data)

    rate = rospy.Rate(0.2) # 200mhz

    while not rospy.is_shutdown():
        rospy.loginfo("{%s} Robostate: %s",
            os.path.splitext(os.path.basename(os.path.abspath(__file__)))[0],
            RobobrainStateHandler.robostate.keys()[RobobrainStateHandler.robostate.values().index(statehandler.state)])

        rate.sleep() # make sure the publish rate maintains at the needed frequency

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
