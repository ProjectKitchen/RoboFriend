#!/usr/bin/env python

# external modules
import signal
import time

# own modules
import python.faceModule as faceModule
import python.rfidModule as rfidModule
import python.webserverModule as webserverModule
import python.statusModule as statusModule
import python.gameCommunicator as gameCommunicator
import python.keyboardModule as keyboardModule
import python.teensyCommunicator as teensyCommunicator

# globals
runFlag = True

def handler_stop_signals(signum, frame):
	global runFlag

	print "*** shutting down ... ***"
	rfidModule.stop()
	webserverModule.stop()
	statusModule.stop()
	gameCommunicator.stop()
	keyboardModule.stop()
	teensyCommunicator.stop()
	faceModule.close()
	runFlag = False

def main():
	global runFlag

	# starting modules
	rfidModule.start()
	webserverModule.start()
	statusModule.start()
	gameCommunicator.start()
	keyboardModule.start()
	faceModule.drawHappyFace()
	print "init done! register signal handlers..."

	# setting up signal handlers for shutdown
	signal.signal(signal.SIGINT, handler_stop_signals)
	signal.signal(signal.SIGTERM, handler_stop_signals)
	print "*** startup completed! ***"

	while runFlag: time.sleep(0.5) # keep program running until stopped

if __name__ == '__main__':
	main()
