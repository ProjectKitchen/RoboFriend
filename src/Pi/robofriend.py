#!/usr/bin/env python

# external modules
import signal
import time
import sys
import os
import rospy
import roslaunch

path = str(os.getcwd()) + "/python"
sys.path.append(path)

# own modules
import faceModule as faceModule
import webserverModule as webserverModule
import statusModule as statusModule
import gameCommunicator as gameCommunicator
import keyboardModule as keyboardModule
import teensyCommunicator as teensyCommunicator
#import ioWarriorModule as ioWarriorModule
#import speechModule as speechModule
#import cam_node as cam_node
#import facedetectionModule as facedetectionModule
from systemModule import *

# import ROS modules
#from ROS_Node.FacedetectionNode import *
from ROS_Node.KeyboardNode import *
#from ROS_Node.MotorNode import *
from ROS_Node.RFIDNode import *
from ROS_Node.SpeechNode import *
from ROS_Node.TeensyNode import *
from ROS_Node.BatteryInfraredNode import *
from ROS_Node.RobobrainNode import *
from ROS_Node.EearsLedNode import *
from ROS_Node.ServoCamNode import *
from ROS_Node.IOWarriorNode import *

# globals
roscore = None
rosrobo = None

def stop():
	global roscore

	print("*** shutting down! ... ***\n\n")
	roscore.terminated()

def handler_stop_signals(signum, frame):
	stop()

def main():
	global roscore, rosrobo

	print("RoboFriend Main Script Ready ...")
	print("Starting Roscore!")
	roscore = Roscore()
	roscore.run()
	print("Starting Robofriend Startup ...")
	rosrobo = RosRobo()
	rosrobo.run()
	
	# TODO: do we need signal handlers?! 
	print("Initialization done! Registering keyboard interrput signal handlers ...")

	# setting up signal handlers for shutdown
	signal.signal(signal.SIGINT, handler_stop_signals)
	signal.signal(signal.SIGTERM, handler_stop_signals)
	print("*** startup completed! ***\n\n")

if __name__ == '__main__':
	main()
