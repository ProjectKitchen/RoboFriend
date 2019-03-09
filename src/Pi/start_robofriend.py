#!/usr/bin/env python

# external modules
import os, sys, time, signal
import rospy

module_path = str(os.getcwd()) + "/python"
sys.path.append(module_path)
catkin_path = str(os.getcwd()) + "/catkin_ws/src/robofriend/scripts"
sys.path.append(catkin_path)

# own modules
import faceModule as faceModule
import webserverModule as webserverModule
import statusModule as statusModule
import gameCommunicator as gameCommunicator
import keyboardModule as keyboardModule
#import teensyCommunicator as teensyCommunicator
#import ioWarriorModule as ioWarriorModule
#import speechModule as speechModule
#import cam_node as cam_node
#import facedetectionModule as facedetectionModule
from systemModule import *

# import ROS modules
from FaceDetectionNode import *
from KeyboardNode import *
from SpeechNode import *
from LedEarsNode import *
from ServoCamNode import *
from IOWarriorNode import *
from VoiceDetectionNode import *

# globals
roscore = None
rosrobo = None

def stop():
	global roscore

	print("*** shutting down! ... ***\n\n")
	try:
		roscore.terminate()
		rospy.signal_shutdown("Shutdown completed!")
	except Exception as inst:
	    rospy.logwarn('*** Roscore termination failed! ***')
	    rospy.logwarn(type(inst))
	    rospy.logwarn(inst.args)

def handler_stop_signals(signum, frame):
	stop()

def main():
	global roscore, rosrobo

	print("RoboFriend Main Script Ready ...")
	print("Starting Roscore!")
	roscore = Roscore()
	roscore.run()

	time.sleep(2)

	print("Starting Robofriend Startup ...")
	rosrobo = RosRobo()
	rosrobo.run()

	# TODO: this has to be handled a better way probably
	rospy.init_node('Robofriend_node', anonymous = True)
	print("Done ... starting Webserver!")
	webserverModule.start()
	print("Done ... starting Gamecommunicator")
	gameCommunicator.start()
	print("Done ... starting KeyboardModule")
	KeyboardNode.node_start()
	print("Done ... starting FaceModue")
	faceModule.drawFace()
	print("Done ... starting RosFacedetectionNode")
	FaceDetectionNode.node_start()
	print("Done ... start Speech Node")
	SpeechNode.node_start()
	print("Done ... start Led Ears Node")
	LedEarsNode.node_start()
	print("Done ... start Servo Camera Node")
	ServoCamNode.node_start()
	print("Done ... start IOWarrior Node")
	IOWarriorNode.node_start()
	print("Done ... start Voice Detection Node")
	VoiceDetectionNode.node_start()
	
	print("Initialization done! Registering keyboard interrput signal handlers ...")

	# setting up signal handlers for shutdown
	signal.signal(signal.SIGINT, handler_stop_signals)
	signal.signal(signal.SIGTERM, handler_stop_signals)
	print("*** startup completed! ***\n\n")

	while not rospy.is_shutdown():
		time.sleep(0.5)

if __name__ == '__main__':
	main()
