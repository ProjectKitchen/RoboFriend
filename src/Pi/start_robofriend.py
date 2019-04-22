#!/usr/bin/env python
import os, rospy, signal, sys, time 

catkin_path = str(os.getcwd()) + "/catkin_ws/src/robofriend/scripts"
sys.path.append(catkin_path)

# import user modules
from SystemModule import *

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
	
	# MOMOKARL: why do we need a node initialization here?
	rospy.init_node('robofriend_node', anonymous = True)

	print(">>>>> startup completed <<<<<\n\n")

	while not rospy.is_shutdown():
		time.sleep(1)

if __name__ == '__main__':
	main()
