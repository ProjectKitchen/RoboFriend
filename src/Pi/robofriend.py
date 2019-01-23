#!/usr/bin/env python

# external modules
import signal
import time
import sys
import os
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
import teensyCommunicator as teensyCommunicator
#import ioWarriorModule as ioWarriorModule
#import speechModule as speechModule
#import cam_node as cam_node
#import facedetectionModule as facedetectionModule
import systemModule as systemModule

# import ROS modules
from FaceDetectionNode import *
from KeyboardNode import *
#from MotorNode import *
from RFIDNode import *
from SpeechNode import *
from TeensyNode import *
from BatteryInfraredNode import *
from RobobrainNode import *
from EarsLedNode import *
from ServoCamNode import *
from IOWarriorNode import *

# globals
runFlag = True

def stop():
	global runFlag

	print("*** shutting down ... ***")
	systemModule.roscore_terminate()
	#rfidModule.stop()
	rfid_node.node_stop()
	webserverModule.stop()
	#statusModule.stop()
	gameCommunicator.stop()
	# keyboardModule.stop()
	keyboard_node.node_stop()
	teensyCommunicator.stop()
	ioWarriorModule.stop()
	# speechModule.stop()
	faceModule.close()
	#cam_node.node_stop()
	robobrain_node.node_stop()
	bat_inf_node.node_stop()
	runFlag = False
	print("*** graceful shutdown completed! ***")

def handler_stop_signals(signum, frame):
	stop()

def main():
	global runFlag

	print("RoboFriend Main Script Ready...")
	print("Starting RosMaster!")
	systemModule.roscore_start()
	print("Initialising RosPy!")
	rospy.init_node('Robofriend_node', anonymous = True)
	print("Done ... starting Robobrain node")
	RobobrainNode.node_start()
	print("Done ... starting RFID!")
	#rfidModule.start()
	RFIDNode.node_start()
	print("Done ... starting Webserver!")
	webserverModule.start()
	print("Done ... starting StatusModule!")
	#statusModule.start()
	print("Done ... starting Gamecommunicator")
	gameCommunicator.start()
	print("Done ... starting KeyboardModule")
	#keyboardModule.start()
	KeyboardNode.node_start()
	print("Done ... starting FaceModue")
	faceModule.drawFace()
	print("Done ... starting RosFacedetectionNode")
	FaceDetectionNode.node_start()
	print("Done ... starting FacedetectListener")
	#facedetectionModule.listener()
	print("Done ... start Speech Node")
	SpeechNode.node_start()
	print("Done ... start Teensy Node")
	TeensyNode.node_start()
	print("Done ... start Battery/Infrared Node")
	BatteryInfraredNode.node_start()
	print("Done ... start Ears/Led Node")
	EarsLedNode.node_start()
	print("Done ... start Servo Camera Node")
	ServoCamNode.node_start()
	print("Done ... start IOWarrior Node")
	IOWarriorNode.node_start()
	print("init done! register signal handlers...")

	# setting up signal handlers for shutdown
	signal.signal(signal.SIGINT, handler_stop_signals)
	signal.signal(signal.SIGTERM, handler_stop_signals)
	print("*** startup completed! ***\n\n")

	#rospy.spin()
	while runFlag: time.sleep(0.5) # keep program running until stopped
	print("[INFO] Main script terminated!!!")

if __name__ == '__main__':
	main()
