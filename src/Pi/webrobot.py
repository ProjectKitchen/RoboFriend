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

""" SETUP """
#setup pygame
pygame.init()

#global
screen = pygame.display.set_mode((654,380))
screen.fill((0, 0, 0))
bat_prozent = 0

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

#opening serial for teensy
try: 
	ser=serial.Serial("/dev/ttyACM0",9600,timeout=1)
	print('***Serial for Teensy opened***')
except:
	sys.exit('Could not open serial for Teensy. Check connection and restart.')

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

#RFIDidentifier
readRFIDnumber = "empty"

#sound setup
if pygame.mixer and not pygame.mixer.get_init():
    print ('Warning, no sound')
    pygame.mixer = None

class dummysound:
	def play(self): pass
	
SendToApp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
	
""" FUNCTIONS """

def chooseAction(data):
	global bat_prozent
	dataArray = data.split(';') # Nachricht wird in ein Array gespeichert
	print(dataArray)
	action = dataArray[0] # erstes Argument
	dataArray = dataArray[1:] # restliche Argumente
	if action == "move":
		move(dataArray)
	elif action == "sound":
		pygame.mixer.stop()
		info = dataArray[0]
		dataArray = dataArray[1:]
		if info == "play":
			playsound(dataArray)
	elif action == "face":
		faceManipulation(dataArray)
	elif action == "battery":
		info = dataArray[0]
		dataArray = dataArray[1:]
		if info == "status": #wenn status abgefragt wird
			ser.write("8")
			bat = str(ser.readline())
			bat = bat.strip(' \t\n\r')
			try:
				bat = int(bat)
			except:
				print("no data available, recieved message: " + str(bat))
			else: #weiterverarbeitung des wertes
				bat_prozent = 0
				if bat > 242:	
					bat_prozent = ((bat-242)/4.95)
					bat_prozent=int(round(bat_prozent))
				sendtogui("battery;"+str(bat_prozent))
				print(bat_prozent)
			print(bat)
	elif action == "IPcheck":
		pass
	
def move(dataArray):
	print(dataArray)
	dir = dataArray[0]
	if len(dataArray) > 1: # step informationen vorhanden, Tablet Toucheingabe verwendet
		print ("step")
		dataArray = dataArray[1:]
		step = dataArray[0] # fuer spaeter hier Erweiterung moeglich auf Laenge der steps eingehen, jetzt nur standardwert verwendet
		if dir == "forward":
			ser.write("7") #zuerst stoppen
			ser.write("1") #forwardstep
		elif dir == "backward":
			ser.write("7") #zuerst stoppen
			ser.write("2") #backwardstep
		elif dir == "left":
			ser.write("7") #zuerst stoppen
			ser.write("4") #leftstep
		elif dir == "right":
			ser.write("7") #zuerst stoppen
			ser.write("3") #rightstep
	else: # keine step informationen vorhanden, daher loop, Joystick verwendet
		print ("loop")
		step = "loop"
		if dir == "forward":
			ser.write("7") #zuerst stoppen
			ser.write("c") #forward loop
		elif dir == "backward":
			ser.write("7") #zuerst stoppen
			ser.write("d") #backward loop
		elif dir == "left":
			ser.write("7") #zuerst stoppen
			ser.write("f") #leftloop
		elif dir == "right":
			ser.write("7") #zuerst stoppen
			ser.write("e") #rightloop
		elif dir == "forward_right":
			ser.write("7") #zuerst stoppen
			ser.write("a") #forward_right loop
		elif dir == "forward_left":
			ser.write("7") #zuerst stoppen
			ser.write("b") #forward_left loop
		elif dir == "backward_right":
			ser.write("7") #zuerst stoppen
			ser.write("5") #backward_right loop
		elif dir == "backward_left":
			ser.write("7") #zuerst stoppen
			ser.write("6") #backward_left loop
		elif dir == "stop":
			ser.write("7") #stoppen

def playsound(dataArray):
	soundname = dataArray[0] #sound(name +) pfad!
	if len(dataArray) > 1: #random sounds oder Aehnliches
		dataArray = dataArray[1:]
		info = dataArray[0]
		if len(dataArray) > 1: # sollten mal noch mehr parameter notwendig sein
			dataArray = dataArray[1:]
			return
		elif info == "random":
			print ("in random")
			r_sounds = ["s1", "s2", "s3", "s4", "s5", "s6", "s7", "s8"]
			rsnr =random.randint(0,7)
			soundpath = soundname+r_sounds[rsnr]+".wav"
		elif info == "mood":
			mood_sounds = ["sad","happy"]
			if sadFace:
				soundpath = soundname+mood_sounds[0]+".wav"
			else:
				soundpath = soundname+mood_sounds[1]+".wav"
	else:
		soundpath = soundname
	
	try:
		selectedSound = pygame.mixer.Sound(soundpath)
	except:
		print ("could not open "+soundpath)
	else:
		selectedSound.play()

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
				print("in elif sadface")
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
			print ("serwrite 9")
			ser.write("9")
	if sadFace == 0:
		DrawFace()
	elif sadFace == 1:
		print("vor drawsadface")
		DrawSadFace()	

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

#when the root IP is selected, return index.html page
@app.route('/')
def index():

	return render_template('index.html', battery = bat_prozent )

#recieve which pin to change from the button press on index.html
#each button returns a number that triggers a command in this function
#
@app.route('/<changepin>', methods=['POST'])
def reroute(changepin):
	global ser
	print(changepin)
	try:
		changePin = urllib.unquote(changepin).decode('utf8') #decode changepin to string
	except:
		print("changepin is not valid")
	else:
		chooseAction(changePin)
	response = make_response(redirect(url_for('index')))
	return(response)

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
				print("neue funktion probieren")
				chooseAction(receivedData)
			else:
				print("alte funktion")
				if newdata == 73:
					continue
				print("vor robomove:" + str(sadFace))
				robomove(newdata)
	finally:
		pygame.quit()

def DrawFace():
	global screen, eyex, eyey, radius
	screen.fill((0, 0, 0))
	pygame.draw.circle(screen, (100,250,250), (163,100), 60) #lefteye
	pygame.draw.circle(screen, (100,250,250), (491,100), 60) #righteye
	pygame.draw.circle(screen, (10,10,10), (163+eyex,100+eyey), 20) #leftpupil
	pygame.draw.circle(screen, (10,10,10), (491+eyex,100+eyey), 20) #rightpupil
	pygame.draw.arc(screen, (100,200,200), (57, -30, 540, 400), 4.7-radius, 4.7+radius, 20) #smile
	pygame.display.flip()
	
def DrawSadFace():
	global screen, eyex, eyey, radius
	screen.fill((0, 0, 0))
	pygame.draw.circle(screen, (100,250,250), (163,100), 60) #lefteye
	pygame.draw.circle(screen, (100,250,250), (491,100), 60) #righteye
	pygame.draw.circle(screen, (10,10,10), (163+eyex,100+eyey), 20) #leftpupil
	pygame.draw.circle(screen, (10,10,10), (491+eyex,100+eyey), 20) #rightpupil
	pygame.draw.arc(screen, (100,200,200), (57, 300, 540, 400), 1.57-radius, 1.57+radius, 20) #smile
	pygame.display.flip()
	
	
def serialRFIDread():
	global readRFIDnumber

	serRFID=serial.Serial("/dev/ttyUSB0",9600)
	print('***Serial for RFID reader opened***')

	try:
		while 1:
			daten = serRFID.read(16)
			daten = daten.replace("\x02", "" )
			daten = daten.replace("\x03", "" )
			daten = daten.replace("\x0a", "" )
			daten = daten.replace("\x0d", "" )
			readRFIDnumber=daten

	finally:
		serRFID.close()

def webserver():
	global app
	# set up the server linstening on port 8765
	app.run(debug=False, host='0.0.0.0', port=8765) 

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
		sys.exit()	
		

def BatteryInfo():
	global bat_prozent
	info = [0, 0, 0]
	sadFace = 0
	bat = ""
	count = 0
	while True:
		count += 1
		chooseAction("battery;status")
		time.sleep(30)
		
def main():
	global readRFIDnumber

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
	
	BatteryThread = threading.Thread(target=BatteryInfo)
	BatteryThread.daemon = True
	BatteryThread.start()

	try:
		while 1:
			event = pygame.event.poll()
			if event.type == pygame.QUIT:
				pygame.quit()
				sys.exit()
			if event.type == pygame.KEYDOWN:
				print('***** Key press recognized ******')
				if event.key == pygame.K_ESCAPE or event.unicode == 'q':
					pygame.quit()
					sys.exit()
			if readRFIDnumber!="empty":
				print('***Serial RFID received:')
				print(readRFIDnumber)
				sendtogui("rfid;"+str(readRFIDnumber))
				readRFIDnumber="empty"	
			time.sleep(0.1)
			
	finally:
		pygame.quit()
		ser.close()

if __name__ == '__main__':
	main()
