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
from systemModule import *

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
