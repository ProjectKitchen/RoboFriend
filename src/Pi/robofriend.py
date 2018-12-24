#!/usr/bin/env python

# external modules
import signal
import time
import sys
import os
import rospy


path = str(os.getcwd()) + "/python"
sys.path.append(path)




# own modules
import faceModule as faceModule
import webserverModule as webserverModule
import statusModule as statusModule
import gameCommunicator as gameCommunicator
import keyboardModule as keyboardModule
import teensyCommunicator as teensyCommunicator
import ioWarriorModule as ioWarriorModule
# import speechModule as speechModule
#import cam_node as cam_node
#import facedetectionModule as facedetectionModule
import systemModule as systemModule

#ROS modules
#from ROS_Node.FacedetectionNode import *
from ROS_Node.KeyboardNode import *
from ROS_Node.MotorNode import *
from ROS_Node.RFIDNode import *
from ROS_Node.SpeechNode import *
from ROS_Node.TeensyNode import *
from ROS_Node.BatteryInfraredNode import *
from ROS_Node.RobobrainNode import *
# from ROS_Node.RobobrainNode import *


# globals
runFlag = True

def stop():
	global runFlag

	print("*** shutting down ... ***")
	systemModule.roscore_terminate()
	#rfidModule.stop()
	rfid_node.node_stop()
	webserverModule.stop()
	statusModule.stop()
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
	robobrain_node.node_start()
	print("Done ... starting RFID!")
	#rfidModule.start()
	rfid_node.node_start()
	print("Done ... starting Webserver!")
	webserverModule.start()
	print("Done ... starting StatusModule!")
	statusModule.start()
	print("Done ... starting Gamecommunicator")
	gameCommunicator.start()
	print("Done ... starting KeyboardModule")
	# keyboardModule.start()
	keyboard_node.node_start()
	print("Done ... starting FaceModue")
	faceModule.drawFace()
	print("Done ... starting RosFacedetectionNode")
	#facedetection_node.node_start()
	print("Done ... starting FacedetectListener")
	#facedetectionModule.listener()
	print("Done ... start Speech Node")
	speech_node.node_start()
	print("Done ... start Teensy Node")
	teensy_node.node_start()
	print("Done ... start Battery/Infrared Node")
	bat_inf_node.node_start()
	print("init done! register signal handlers...")

	# setting up signal handlers for shutdown
	signal.signal(signal.SIGINT, handler_stop_signals)
	signal.signal(signal.SIGTERM, handler_stop_signals)
	print("*** startup completed! ***")

	#rospy.spin()
	while runFlag: time.sleep(0.5) # keep program running until stopped
	print("[INFO] Main script terminated!!!")

if __name__ == '__main__':
	main()
