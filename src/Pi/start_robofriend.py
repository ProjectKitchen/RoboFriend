#!/usr/bin/env python
import os, rospy, signal, sys, time 

# module_path = str(os.getcwd()) + "/python"
# sys.path.append(module_path)
catkin_path = str(os.getcwd()) + "/catkin_ws/src/robofriend/scripts"
sys.path.append(catkin_path)

# import user modules
from SystemModule import *
# TODO: we can probably get rid of all these imports
# import faceModule as faceModule
# import statusModule as statusModule
#from FaceDetectionNode import *
# from KeyboardNode import *
# from SpeechNode import *
# from LedEarsNode import *
# from ServoCamNode import *
# from IOWarriorNode import *
# from VoiceDetectionNode import *

# globals
roscore = None
rosrobo = None

def stop():
	global roscore

	print(">>>>> shutting down <<<<<")
	try:
		roscore.terminate()
		rospy.signal_shutdown("shutdown completed!")
	except Exception as inst:
		rospy.logwarn('*** this is a controlled catch. ***')
		rospy.logwarn('*** roscore termination failed. ***')
		rospy.logwarn(type(inst))
		rospy.logwarn(inst.args[1])
	print(">>>>> shutdown successful <<<<<")

def handler_stop_signals(signum, frame):
	stop()

def main():
	global roscore, rosrobo

	print(">>>>> robofriend main script ready <<<<<")
	print(">>>>> registering keyboard interrput signal handlers <<<<<")

	# setting up signal handlers for shutdown
	signal.signal(signal.SIGINT, handler_stop_signals)
	signal.signal(signal.SIGTERM, handler_stop_signals)

	print(">>>>> starting roscore <<<<<")
	roscore = Roscore()
	roscore.run()

	time.sleep(2)

	print(">>>>> starting robofriend startup <<<<<")
	rosrobo = RosRobo()
	rosrobo.run()
	
	# TODO: we can probably get rid of all these imports
	# MOMOKARL: why do we need a node initialization here?
	rospy.init_node('robofriend_node', anonymous = True)
# 	print("Done ... starting KeyboardModule")
# 	KeyboardNode.node_start()
# 	print("Done ... starting FaceModue")
# 	faceModule.drawFace()
# 	print("Done ... starting RosFacedetectionNode")
# 	#FaceDetectionNode.node_start()
# 	print("Done ... start Speech Node")
# 	SpeechNode.node_start()
# 	print("Done ... start Led Ears Node")
# 	LedEarsNode.node_start()
# 	print("Done ... start Servo Camera Node")
# 	ServoCamNode.node_start()
# 	print("Done ... start IOWarrior Node")
# 	IOWarriorNode.node_start()
# 	print("Done ... start Voice Detection Node")
# 	# VoiceDetectionNode.node_start()
	print(">>>>> startup completed <<<<<\n\n")

	while not rospy.is_shutdown():
		time.sleep(1)

if __name__ == '__main__':
	main()
