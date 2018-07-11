#!/usr/bin/env python

# external modules
import signal

# own modules
import python.faceModule as faceModule
import python.rfidModule as rfidModule
import python.webserverModule as webserverModule
import python.statusModule as statusModule
import python.gameCommunicator as gameCommunicator
import python.keyboardModule as keyboardModule
import python.teensyCommunicator as teensyCommunicator

def handler_stop_signals(signum, frame):
	print "*** shutting down ... ***"
	rfidModule.stop()
	webserverModule.stop()
	statusModule.stop()
	gameCommunicator.stop()
	keyboardModule.stop()
	teensyCommunicator.stop()
	faceModule.close()

def main():
	# starting modules
	rfidModule.start()
	webserverModule.start()
	statusModule.start()
	gameCommunicator.start()
	keyboardModule.start()
	faceModule.drawHappyFace()

	# setting up signal handlers for shutdown
	signal.signal(signal.SIGINT, handler_stop_signals)
	signal.signal(signal.SIGTERM, handler_stop_signals)

if __name__ == '__main__':
	main()
