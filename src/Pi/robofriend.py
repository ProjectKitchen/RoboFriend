#!/usr/bin/env python
# neues Kommunikationsprotokoll implementiert
"""Proof of concept gfxdraw example"""

import pygame
import pygame.gfxdraw
import socket
import sys
import threading
import serial
import time
import curses
import random, os.path
from time import sleep
from flask import Flask, render_template, request, redirect, url_for, make_response
import urllib
import pyttsx
#import pyautogui

#own modules
import python.teensyCommunicator as teensySender

""" SETUP """

#pyautogui.FAILSAFE = False
#pyautogui.moveTo(0, 0)

#switch on head, LEDS and Servo
p=os.system('sudo ./iowarrior/iow 0 10 10 140')

#setup pygame
pygame.init()

#setup pyttsx / espeak
speechEngine = pyttsx.init()
speechEngine.setProperty('rate', 150) #150 words per minute
speechEngine.say('i am robofriend')
speechEngine.runAndWait()

#global
#screen = pygame.display.set_mode((654,380))
screen = pygame.display.set_mode((654, 380), pygame.FULLSCREEN)
screen.fill((0, 0, 0))
bat_prozent = 0
bat_prozent_lock = threading.Lock()
cameraPos = 140
earColorR = 0
earColorG = 10
earColorB = 10


#face informationen
radius = 0.8
eyex = 0
eyey = 0
sadFace = 0
eyestep=10
eyestep = 10

#directory manager (eg. for sound)
main_dir = os.path.split(os.path.abspath(__file__))[0]

#set up flask server
app = Flask(__name__)

teensySender.init()

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

#sound setup
if pygame.mixer and not pygame.mixer.get_init():
    print ('Warning, no sound')
    pygame.mixer = None

class dummysound:
	def play(self): pass
	
SendToApp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
	
""" FUNCTIONS """
#loading sounds from subdir 'data'    
def load_sound(file):
	if not pygame.mixer: return dummysound()
	file = os.path.join(main_dir, 'data', file)
	try:
		sound = pygame.mixer.Sound(file)
		return sound
	except pygame.error:
		print ('Warning, unable to load, %s' % file)
	return dummysound()

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

	return render_template('index.html', battery = bat_prozent )

@app.route('/camera/up', methods=['POST'])
def cameraup():
	global cameraPos
	#10 - 160
	if cameraPos <= 150:
		cameraPos += 10
	setCameraPos(cameraPos)
	return("OK")

@app.route('/camera/down', methods=['POST'])
def cameradown():
	global cameraPos
	#10 - 160
	if cameraPos >= 20:
		cameraPos -= 10
	setCameraPos(cameraPos)
	return("OK")

@app.route('/ear/color/<earColorR>/<earColorG>/<earColorB>', methods=['POST'])
def setEarRGB(earColorR, earColorG, earColorB):
	global cameraPos
	r = translateIntRange(int(earColorR), 0, 255, 0, 15)
	g = translateIntRange(int(earColorG), 0, 255, 0, 15)
	b = translateIntRange(int(earColorB), 0, 255, 0, 15)
	sendToIOWarrior(r, g, b, cameraPos)
	return("OK")

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
	response = make_response(redirect(url_for('index')))
	return(response)

# this functions handles input from the gamegui, as well as webserver and BatteryThread-thread
def chooseAction(data):
	global bat_prozent, bat_prozent_lock
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
		pygame.mixer.stop()
		info = dataArray[0]
		dataArray = dataArray[1:]
		if info == "play":
			playsound(dataArray)
	elif action == "face":
		faceManipulation(dataArray)
	elif action == "get":
		info = dataArray[0]
		dataArray = dataArray[1:]
		if info == "status": #wenn status abgefragt wird
			status = teensySender.getStatus()
			statusArray=status.split(',')
			if (statusArray[0] == "Sensors"):
				bat = int (statusArray[1])
				irSensorLeft= int (statusArray[2])
				irSensorMiddle= int (statusArray[3])
				irSensorRight= int (statusArray[4])
				print ("Battery="+str(bat)+"("+str(bat/20)+" Volt)")
				if (bat<630):
					print ("LOW BATTERY !! - Please Recharge!!")
				print ("IRSensors="+str(irSensorLeft)+"/"+str(irSensorMiddle)+"/"+str(irSensorRight))
				sendtogui("battery;"+str(bat/20))

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

# this function is called by chooseAction if the robot has to speak/make a sound			
def playsound(dataArray):
	soundname = dataArray[0] #sound(name +) pfad!
	if len(dataArray) > 1: #random sounds oder Aehnliches
		dataArray = dataArray[1:]
		info = dataArray[0]
		print ("info = " + info)

		if len(dataArray) > 1: # sollten mal noch mehr parameter notwendig sein
			dataArray = dataArray[1:]
			return
		elif info == "random":
			print ("random sound")
			r_sounds = ["s1", "s2", "s3", "s4", "s5", "s6", "s7", "s8"]
			rsnr =random.randint(0,7)
			soundpath = soundname+r_sounds[rsnr]+".wav"
		elif info == "mood":
			print ("mood sound")
			mood_sounds = ["sad","happy"]
			if sadFace:
				soundpath = soundname+mood_sounds[0]+".wav"
			else:
				soundpath = soundname+mood_sounds[1]+".wav"
	else:
		print ("playing soundfile " + soundname)
		soundpath = soundname
	
	try:
		print ("playing sound "+soundpath)
		selectedSound = pygame.mixer.Sound(soundpath)
	except:
		print ("could not open "+soundpath)
	else:
		selectedSound.play()

# this function is called by chooseAction if the facial expression of the robot has to change
def faceManipulation(dataArray):
	global radius, eyey, eyex, sadFace, eyestep
	faceObject = dataArray[0] #eyes or smile
	dataArray = dataArray[1:]
	if faceObject == "smile":
		changing = dataArray[0]
		if changing == "increase":
			if (radius < 1.3 and sadFace == 0):
				radius=radius+0.1
			elif (sadFace ==1):
				if radius < 0.3:
					sadFace = 0
					radius = radius +0.1
				else:
					sadFace = 1
					radius = radius-0.1				
		elif changing == "decrease":
			if (radius > 0.3 and sadFace == 0):
				radius=radius-0.1
			elif ((sadFace==1 or radius < 0.3) and radius < 0.8):
				sadFace=1
				radius=radius+0.1
	elif faceObject == "eyes":
		changing = dataArray[0]
		if changing == "up":
			if eyey > -40:
				eyey=eyey-eyestep
		elif changing == "down":
			if eyey < 40:
				eyey=eyey+eyestep
		elif changing == "right":
			if eyex < 40:
				eyex=eyex+eyestep
		elif changing == "left":
			if eyex > -40:
				eyex=eyex-eyestep
	elif faceObject == "answer":
		changing = dataArray[0]
		if changing == "correct":
			sadFace = 0
			radius = 0.8
		if changing == "wrong":
			sadFace = 1
			radius = 0.8
			teensySender.shakeHeadForNo()
	if sadFace == 0:
		DrawFace()
	elif sadFace == 1:
		DrawSadFace()	

# Updating Facial expression of robot in case of a happy expression
def DrawFace():
	global screen, eyex, eyey, radius
	screen.fill((0, 0, 0))
	pygame.draw.circle(screen, (100,250,250), (163,100), 60) #lefteye
	pygame.draw.circle(screen, (100,250,250), (491,100), 60) #righteye
	pygame.draw.circle(screen, (10,10,10), (163+eyex,100+eyey), 20) #leftpupil
	pygame.draw.circle(screen, (10,10,10), (491+eyex,100+eyey), 20) #rightpupil
	pygame.draw.arc(screen, (100,200,200), (57, -30, 540, 400), 4.7-radius, 4.7+radius, 20) #smile
	pygame.display.flip()

# Updating Facial expression of robot in case of a sad expression	
def DrawSadFace():
	global screen, eyex, eyey, radius
	screen.fill((0, 0, 0))
	pygame.draw.circle(screen, (100,250,250), (163,100), 60) #lefteye
	pygame.draw.circle(screen, (100,250,250), (491,100), 60) #righteye
	pygame.draw.circle(screen, (10,10,10), (163+eyex,100+eyey), 20) #leftpupil
	pygame.draw.circle(screen, (10,10,10), (491+eyex,100+eyey), 20) #rightpupil
	pygame.draw.arc(screen, (100,200,200), (57, 300, 540, 400), 1.57-radius, 1.57+radius, 20) #smile
	pygame.display.flip()

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
	global screen, IP, radius, eyex, eyey, sadFace
	
	#Connect the UDP_Port
	try:
		sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
		sock.bind((UDP_IP, UDP_PORT))
		print('***** UDP SOCKET opened ******')
	except:
		print('Could not open UDP SOCKET!')

	DrawFace()
	
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
	while True:
		chooseAction("get;status")
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
					chooseAction("face;smile;increase");
				if event.unicode == '2':
					print('smile decrease via keyboard')
					chooseAction("face;smile;decrease");
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
					chooseAction("sound;play;data/fabibox/;random");
				if event.unicode == '4':
					print('sound play mood via keyboard')
					chooseAction("sound;play;data/;mood");
			time.sleep(0.1)

	except Exception, e:
		print ('Exception:'+ str(e))
	finally:
		pygame.quit()
		teensySender.close()

if __name__ == '__main__':
	main()
