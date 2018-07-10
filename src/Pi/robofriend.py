#!/usr/bin/env python

import pygame
import socket
import sys
import threading
import serial
import time
import curses
from time import sleep
from flask import Flask, render_template, request, redirect, url_for, make_response, send_file
import urllib
import pyttsx
import json
#import pyautogui

#own modules
import python.teensyCommunicator as teensySender
import python.faceModule as faceModule
import python.soundModule as soundModule

faceModule.drawHappyFace()

""" SETUP """

#pyautogui.FAILSAFE = False
#pyautogui.moveTo(0, 0)

#switch on head, LEDS and Servo
p=os.system('sudo ./iowarrior/iow 0 10 10 140')

#setup pyttsx / espeak
speechEngine = pyttsx.init()
speechEngine.setProperty('rate', 150) #150 words per minute
speechEngine.say('i am robofriend')
speechEngine.runAndWait()

#global
cameraPos = 140
earColorR = 0
earColorG = 10
earColorB = 10
currentStatus = None

#set up flask server
app = Flask(__name__)

UDP_PORT = 9000 #socket port

#Handle command line arguments to get IP address
if (len(sys.argv) == 2):
	try:
		UDP_IP = sys.argv[1]
		socket.inet_aton(UDP_IP)
	except:
		sys.exit('Invalid IP address, Try again')
else:
		UDP_IP = ''
IP = ''

SendToApp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

def setCameraPos(pos):
	global earColorR, earColorG, earColorB
	sendToIOWarrior(earColorR, earColorG, earColorB, pos)

def sendToIOWarrior(earColorR, earColorG, earColorB, cameraPos):
	os.system('sudo ./iowarrior/iow ' + str(earColorR) + ' ' + str(earColorG) + ' ' + str(earColorB) + ' ' + str(cameraPos))

def translateIntRange(value, leftMin, leftMax, rightMin, rightMax):
	# Figure out how 'wide' each range is
	leftSpan = leftMax - leftMin
	rightSpan = rightMax - rightMin

	# Convert the left range into a 0-1 range (float)
	valueScaled = float(value - leftMin) / float(leftSpan)

	# Convert the 0-1 range into a value in the right range.
	return abs(rightMin + (valueScaled * rightSpan))
	
#when the root IP is selected, return index.html page
@app.route('/')
def index():
	return make_response(send_file('index.html'))

@app.route('/camera/up', methods=['POST'])
def cameraup():
	global cameraPos
	#10 - 160
	if cameraPos <= 150:
		cameraPos += 10
	setCameraPos(cameraPos)
	return getResponse("OK")

@app.route('/camera/down', methods=['POST'])
def cameradown():
	global cameraPos
	#10 - 160
	if cameraPos >= 20:
		cameraPos -= 10
	setCameraPos(cameraPos)
	return getResponse("OK")

@app.route('/ear/color/<earColorR>/<earColorG>/<earColorB>', methods=['POST'])
def setEarRGB(earColorR, earColorG, earColorB):
	global cameraPos
	r = translateIntRange(int(earColorR), 0, 255, 0, 15)
	g = translateIntRange(int(earColorG), 0, 255, 0, 15)
	b = translateIntRange(int(earColorB), 0, 255, 0, 15)
	sendToIOWarrior(r, g, b, cameraPos)
	return getResponse("OK")

@app.route('/move/<left>/<right>/<duration>', methods=['POST'])
def move(left, right, duration):
	teensySender.move(left, right, duration)
	return getResponse("OK")

@app.route('/get/status', methods=['GET'])
def getStatus():
	global currentStatus
	return getResponse(json.dumps(currentStatus))

@app.route('/<action>', methods=['POST'])
# webserver rerouting - action indicates the chosen command which will be decoded and then interpreted with the function chooseAction 
def reroute(action):
	print(action)
	try:
		action = urllib.unquote(action).decode('utf8') #decode action to string
	except:
		print("action is not valid")
	else:
		chooseAction(action)
	return getResponse("OK")

def getResponse(responseString):
    resp = make_response(responseString)
    resp.headers['Access-Control-Allow-Origin'] = '*'
    return resp

# this functions handles input from the gamegui, as well as webserver and BatteryThread-thread
def chooseAction(data):
	global currentStatus
	dataArray = data.split(';') # Nachricht wird in ein Array gespeichert
	print(dataArray)
	action = dataArray[0] # erstes Argument
	dataArray = dataArray[1:] # restliche Argumente
	if action == "move":
		move(dataArray)
	elif action == "say":
		print('saying :' + dataArray[0])
		speechEngine.say(dataArray[0])
		speechEngine.runAndWait()
	elif action == "sound":
		info = dataArray[0]
		dataArray = dataArray[1:]
		if info == "play":
			soundModule.playsound(dataArray)
	elif action == "face":
		faceModule.faceManipulation(dataArray)
	elif action == "get":
		info = dataArray[0]
		if info == "status" and currentStatus: #wenn status abgefragt wird
			sendtogui("battery;"+str(currentStatus['batVolt']))
	elif action == "IPcheck":
		pass

# this function is called by chooseAction if the robot has to move
def move(dataArray):
	print(dataArray)
	dir = dataArray[0]
	if len(dataArray) > 1: # step informationen vorhanden, Tablet Toucheingabe verwendet
		print ("step")
		dataArray = dataArray[1:]
		step = dataArray[0] # fuer spaeter hier Erweiterung moeglich auf Laenge der steps eingehen, jetzt nur standardwert verwendet
		if dir == "forward":
			teensySender.moveForwardStep()
		elif dir == "backward":
			teensySender.moveBackStep()
		elif dir == "left":
			teensySender.moveLeftStep()
		elif dir == "right":
			teensySender.moveRightStep()
	else: # keine step informationen vorhanden, daher loop, Joystick verwendet
		print ("loop")
		if dir == "forward":
			teensySender.moveForwardLoop()
		elif dir == "backward":
			teensySender.moveBackLoop()
		elif dir == "left":
			teensySender.moveLeftLoop()
		elif dir == "right":
			teensySender.moveRightLoop()
		elif dir == "forward_right":
			teensySender.moveForwardRightLoop()
		elif dir == "forward_left":
			teensySender.moveForwardLeftLoop()
		elif dir == "backward_right":
			teensySender.moveBackRightLoop()
		elif dir == "backward_left":
			teensySender.moveBackLeftLoop()
		elif dir == "stop":
			teensySender.stopMovement()

# This function is used to send information to the gamegui and can be called by the Thread serialRFIDread (for RFID data) and chooseAction (for battery information)
def sendtogui(i):	
	global UDP_PORT, SendToApp, IP
	#IP = '10.10.10.111'
	if IP == '':
		return
	BytesToSend = bytes(":RUN:" + str(i) + ":EOL:")
	try:
		SendToApp.sendto(BytesToSend, (IP, UDP_PORT+1))
		print(BytesToSend)
	except socket.error, msg:
		print 'Error Code: ' + str(msg[0]) + 'Message ' + msg[1]
		IP = ''
		
# This function is used as Thread to always listen if there is a client (gamegui) sending commands
def data_listener():
	global IP
	
	#Connect the UDP_Port
	try:
		sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
		sock.bind((UDP_IP, UDP_PORT))
		print('***** UDP SOCKET opened ******')
	except:
		print('Could not open UDP SOCKET!')
	
	try:
		startFlag = ":RUN:"
		endFlag = ":EOL:"
		receivedData = ""
		while 1:
			tempData, addr = sock.recvfrom(50)
			receivedData += tempData
			startFlagIndex = receivedData.find(startFlag)
			if startFlagIndex != -1:
				receivedData = receivedData[startFlagIndex + len(startFlag):]

			endFlagIndex = receivedData.find(endFlag)
			if endFlagIndex == -1:
				continue
			
			receivedData = receivedData[:endFlagIndex] # ab dieser Zeile ist nur noch Befehlskette da (ohne start/endflag)
			
			print('***** received data from app ******')			
			IP = addr[0]
			print (addr)
			
			try:
				newdata = int(receivedData)
			except:
				chooseAction(receivedData)
			else:
				if newdata == 73:
					continue
				robomove(newdata)
	finally:
		pygame.quit()

# This function is used as Thread to always listen if there is incoming RFID data
def serialRFIDread():
	readRFIDnumber = "empty"

	serRFID=serial.Serial("/dev/ttyUSB0",9600)
	print('***Serial for RFID reader opened***')

	try:
		while 1:
			daten = serRFID.read(16)
			daten = daten.replace("\x02", "" )
			daten = daten.replace("\x03", "" )
			daten = daten.replace("\x0a", "" )
			daten = daten.replace("\x0d", "" )
			#lock
			readRFIDnumber=daten
			#release
			if readRFIDnumber!="empty": #if rfid not locked: if rfid != empty, lock. --- release
				print('***Serial RFID received:')
				print(readRFIDnumber)
				sendtogui("rfid;"+str(readRFIDnumber))
				readRFIDnumber="empty"	
	finally:
		serRFID.close()

# This function is used as Thread to open and handle the webserver
def webserver():
	global app
	# set up the server linstening on port 8765
	app.run(debug=False, host='0.0.0.0', port=8765) 

# This function is used as Thread to periodically update the battery information
def StatusInfo():
	global currentStatus
	while True:
		currentStatus = teensySender.getStatus()
		print ("Battery= " + str(currentStatus['batVolt']) + " Volt)")
		if currentStatus['batVolt'] < 31.5:
			print ("LOW BATTERY !! - Please Recharge!!")
		print ("IRSensors="+str(currentStatus['irLeft'])+"/"+str(currentStatus['irMiddle'])+"/"+str(currentStatus['irRight']))
		time.sleep(1)
	
# Main function starts all threads and handles event interrupts for closing this program	
def main():
	
	AppListener = threading.Thread(target=data_listener)
	AppListener.daemon = True
	AppListener.start()

	print('***** BEFORE Webserver Start ******')
	WebserverThread = threading.Thread(target=webserver)
	WebserverThread.daemon = True
	WebserverThread.start()
	print('***** AFTER Webserver Start ******')
	
	RFIDReader = threading.Thread(target=serialRFIDread)
	RFIDReader.daemon = True
	RFIDReader.start()
	
	StatusThread = threading.Thread(target=StatusInfo)
	StatusThread.daemon = True
	StatusThread.start()

	try:
		while 1:
			event = pygame.event.poll()
			if event.type == pygame.QUIT:
				pygame.quit()
				sys.exit()
			if event.type == pygame.KEYDOWN:
				print('***** Key press recognized ******')
				if event.key == pygame.K_ESCAPE or event.unicode == 'q':
					print('quit via keyboard')
					pygame.quit()
					sys.exit()
				if event.unicode == '1':
					print('smile increase via keyboard')
					faceModule.increaseSmile()
				if event.unicode == '2':
					print('smile decrease via keyboard')
					faceModule.decreaseSmile()
				if event.key == pygame.K_UP:
					print("move forward via keyboard")
					teensySender.moveForwardLoop()
				if event.key == pygame.K_DOWN:
					print('move back via keyboard')
					teensySender.moveBackLoop()
				if event.key == pygame.K_LEFT:
					print("move left via keyboard")
					teensySender.moveLeftLoop()
				if event.key == pygame.K_RIGHT:
					print('move right via keyboard')
					teensySender.moveRightLoop()
				if event.key == pygame.K_RETURN:
					print('move stop via keyboard')
					teensySender.stopMovement()
				if event.unicode == '3':
					print('sound play random via keyboard')
					soundModule.playRandom()
				if event.unicode == '4':
					print('sound play mood via keyboard')
					soundModule.playMood()
			time.sleep(0.1)

	except Exception, e:
		print ('Exception:'+ str(e))
	finally:
		pygame.quit()
		teensySender.close()

if __name__ == '__main__':
	main()
