#!/usr/bin/env python
# version from 24.06.17 15:00
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
#import tcomm
from time import sleep
from flask import Flask, render_template, request, redirect, url_for, make_response

""" SETUP """
#setup pygame
pygame.init()

#global
screen = pygame.display.set_mode((654,380))
screen.fill((0, 0, 0))
bat_prozent = 0

#drive status:
F = 0 #forward
R = 0 #right
L = 0 #left
B = 0 #back


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

#data = [] #no idea where its used or what for
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

#Sounds 
try:
	GameOne = pygame.mixer.Sound('data/spiel1.wav')
	BlueObj = pygame.mixer.Sound('data/blau.wav') #
	GreenObj = pygame.mixer.Sound('data/gruen.wav') #
	YellowObj = pygame.mixer.Sound('data/gelb.wav') #
	RedObj = pygame.mixer.Sound('data/rot.wav') #
	quader = pygame.mixer.Sound('data/quader.wav')
	wrong = pygame.mixer.Sound('data/wrong.wav')
	wuerfel = pygame.mixer.Sound('data/wuerfel.wav')
	zyl = pygame.mixer.Sound('data/zyl.wav')
	correct = pygame.mixer.Sound('data/correct.wav') #
	GameTwo = pygame.mixer.Sound('data/robogame2.wav') #
	GTS1 = pygame.mixer.Sound('data/2robory1.wav') #
	GTS2 = pygame.mixer.Sound('data/2robory2.wav') #
	GTS3 = pygame.mixer.Sound('data/2robory3.wav') #
	GTS4 = pygame.mixer.Sound('data/2robory4.wav') #
	GTS5 = pygame.mixer.Sound('data/2robory5.wav') #
	GTS6 = pygame.mixer.Sound('data/2robory6.wav') #
	GameThree = pygame.mixer.Sound('data/robogame3.wav') #
	sheep = pygame.mixer.Sound('data/sheep.wav')
	cat = pygame.mixer.Sound('data/cat.wav')
	cow = pygame.mixer.Sound('data/cow.wav') #
	dogs = pygame.mixer.Sound('data/dogs.wav') #
	elephant = pygame.mixer.Sound('data/elephant.wav')
	bird = pygame.mixer.Sound('data/bird.wav') #
	sheepR = pygame.mixer.Sound('data/robeep.wav') #
	catR = pygame.mixer.Sound('data/robocatser.wav') #
	cowR = pygame.mixer.Sound('data/robocow.wav') #
	dogsR = pygame.mixer.Sound('data/robodogs.wav') #
	elephantR = pygame.mixer.Sound('data/robofant.wav') #
	birdR = pygame.mixer.Sound('data/robobird.wav') #
	happy = pygame.mixer.Sound('data/happy.wav')
	sad = pygame.mixer.Sound('data/sad.wav')
	GameFour = pygame.mixer.Sound('data/spiel4.wav') #
	NotAnAnimal = pygame.mixer.Sound('data/keinTier.wav') #
	#cat = pygame.mixer.Sound('data/robocatser.wav')
except:	
	print("Could not open all sounds")
	
SendToApp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
	
	
""" FUNCTIONS """

#assigning actions to SOCKET activity
def robomove(act, info, sadFace):
	global screen, bat_prozent, R, F, L, B
	radius= info[0]
	eyex= info[1]
	eyey= info[2]
	eyestep=10
	eyestep = 10
	#sadFace = 0
	if act != 0:
		pygame.mixer.stop()
	print ("robomove called with:" +str(act))
	
	if act == 0:
		ser.write("8")
		bat = str(ser.readline())
		bat = bat.strip(' \t\n\r')
		try:
			bat = int(bat)
			#weiterverarbeitung des wertes
		except:
			print("no data available, recieved message: " + str(bat))
		else:
			bat_prozent = 0
			if bat > 242:	
				bat_prozent = ((bat-242)/4.95)
				bat_prozent=int(round(bat_prozent))
			sendtogui(bat_prozent)
			print(bat_prozent)
		print(bat)
		
	if act == 1: # FW step
		#send stop
		B = 0
		L = 0
		F = 0
		R = 0
		ser.write("7")
		#tcomm.tcomm(ser)
		#send F
		ser.write("1")
		#tcomm.tcomm(ser)
		#F = 1
	if act == 2: # BW step
		#send stop
		B = 0
		L = 0
		F = 0
		R = 0
		ser.write("7")
		#tcomm.tcomm(ser)
		#send B
		ser.write("2")
		#tcomm.tcomm(ser)
		#B = 1
	if act == 3: # RECHTS 
		#send stop
		B = 0
		L = 0
		F = 0
		R = 0
		ser.write("7")
		#tcomm.tcomm(ser)
		#send R
		ser.write("3")
		#tcomm.tcomm(ser)
		#R = 1
		#F = 1
	if act == 4: # LINKS 
		#send stop
		B = 0
		L = 0
		F = 0
		R = 0
		ser.write("7")
		#tcomm.tcomm(ser)
		#send R
		ser.write("4")
		#tcomm.tcomm(ser)
		#L = 1
		#F = 1
	if act == 5:
		if (radius < 1.3 and sadFace == 0):
			radius=radius+0.1
		elif (sadFace ==1):
			if radius < 0.3:
				sadFace = 0
				radius = radius +0.1			
			else:
				sadFace = 1
				radius = radius-0.1	
		print(radius)
	if act == 6:
		if (radius > 0.3 and sadFace == 0):
			radius=radius-0.1
		elif ((sadFace==1 or radius < 0.3) and radius < 0.8):
			print("in elif sadface")
			sadFace=1
			radius=radius+0.1
		print(sadFace)
		print(radius)
	if act == 7:
		if eyey > -40:
			eyey=eyey-eyestep
	if act == 8:
		if eyey < 40:
			eyey=eyey+eyestep
	if act == 9:
		if eyex > -40:
			eyex=eyex-eyestep
	if act == 10:
		if eyex < 40:
			eyex=eyex+eyestep
	if act == 11:
		GameOne.play()
	if act == 12:
		correct.play()
		sadFace = 0
		radius = 0.8
	if act == 13:
		wrong.play()
		sadFace = 1
		radius = 0.8
		print ("serwrite 9")
		ser.write("9")
		#tcomm.tcomm(ser)
	if act == 14:
		BlueObj.play()
	if act == 15:
		YellowObj.play()		
	if act == 16:
		GreenObj.play()
	if act == 17:
		RedObj.play()
	if act == 18:
		zyl.play()
	if act == 19:
		quader.play()
	if act == 20:
		wuerfel.play()
	if act == 21:
		GameTwo.play()
	if act == 22:
		GTS1.play()
	if act == 23:
		GTS2.play()
	if act == 24:
		GTS3.play()
	if act == 25:
		GTS4.play()
	if act == 26:
		GTS5.play()
	if act == 27:
		GTS6.play()	
	if act == 28:
		GameThree.play()
	if act == 29:
		bird.play()
	if act == 30:
		cat.play()	
	if act == 31:
		cow.play()	
	if act == 32:
		dogs.play()
	if act == 33:
		elephant.play()	
	if act == 34:
		sheep.play()	
	if act == 35:
		birdR.play()
		sadFace = 0
		radius = 0.8
	if act == 36:
		catR.play()	
		sadFace = 0
		radius = 0.8
	if act == 37:
		cowR.play()
		sadFace = 0
		radius = 0.8
	if act == 38:
		dogsR.play()
		sadFace = 0
		radius = 0.8
	if act == 39:
		elephantR.play()
		sadFace = 0
		radius = 0.8
	if act == 40:
		sheepR.play()
		sadFace = 0
		radius = 0.8
	if act == 41:
		GameFour.play()
	if act == 42:
		# no animal
		NotAnAnimal.play()
	if act == 43: # rechts loop
		if (R ==1) and (F == 0) and (L == 0) and (B == 0):
			#maybe sensorwert updaten
			pass
		else:
			#send stop
			B = 0
			L = 0
			F = 0
			R = 0
			ser.write("7")
			#tcomm.tcomm(ser)
			#send R
			ser.write("e")
			#tcomm.tcomm(ser)
			R = 1
	if act == 44: # links hinten
		if (L ==1) and (B == 1) and (F == 0) and (R == 0):
			#maybe sensorwert updaten
			pass
		else:
			#send stop
			B = 0
			L = 0
			F = 0
			R = 0
			ser.write("7")
			#tcomm.tcomm(ser)
			#send LBW
			ser.write("6")
			#tcomm.tcomm(ser)
			L = 1
			B = 1
	if act == 45: # links loop
		if (R == 0) and (F == 0) and (L == 1) and (B == 0):
			#maybe sensorwert updaten
			pass
		else:
			#send stop
			B = 0
			L = 0
			F = 0
			R = 0
			ser.write("7")
			#tcomm.tcomm(ser)
			#send L
			ser.write("f")
			#tcomm.tcomm(ser)
			L = 1
	if act == 46: # rechts hinten
		if (R ==1) and (F == 0) and (L == 0) and (B == 1):
			#maybe sensorwert updaten
			pass
		else:
			#send stop
			B = 0
			L = 0
			F = 0
			R = 0
			ser.write("7")
			#send RBW
			ser.write("5")
			#tcomm.tcomm(ser)
			R = 1
			B = 1
	if act == 47: # stop
		#if R or F or L or B:
		#send stop
		B = 0
		L = 0
		F = 0
		R = 0
		print("serwrite")
		ser.write("7")
		#tcomm.tcomm(ser)
	if act == 48: # FW
		if (R == 0) and (F == 1) and (L == 0) and (B == 0):
			#maybe sensorwert updaten
			pass
		else:
			#send stop
			B = 0
			L = 0
			F = 0
			R = 0
			ser.write("7")
			#tcomm.tcomm(ser)
			#send R
			ser.write("c")
			#tcomm.tcomm(ser)
			F = 1
	if act == 49: #BW
		if (R ==0) and (F == 0) and (L == 0) and (B == 1):
			#maybe sensorwert updaten
			pass
		else:
			#send stop
			B = 0
			L = 0
			F = 0
			R = 0
			ser.write("7")
			#tcomm.tcomm(ser)
			#send R
			ser.write("d")
			#tcomm.tcomm(ser)
			B = 1
	if act == 50: # rechts vor
		if (R ==1) and (F == 1) and (L == 0) and (B == 0):
			#maybe sensorwert updaten
			pass
		else:
			#send stop
			B = 0
			L = 0
			F = 0
			R = 0
			ser.write("7")
			#tcomm.tcomm(ser)
			#send R
			ser.write("a")
			#tcomm.tcomm(ser)
			R = 1
			F = 1
	if act == 51: #links vor
		if (R ==0) and (F == 1) and (L == 1) and (B == 0):
			#maybe sensorwert updaten
			pass
		else:
			#send stop
			B = 0
			L = 0
			F = 0
			R = 0
			ser.write("7")
			#tcomm.tcomm(ser)
			#send R
			ser.write("b")
			#tcomm.tcomm(ser)
			L = 1
			F = 1
	if act == 52: #random sound
		r1 = "data/s1.wav"
		r2 = "data/s2.wav"
		r3 = "data/s3.wav"
		r4 = "data/s4.wav"
		r5 = "data/s5.wav"
		r6 = "data/s6.wav"
		r7 = "data/s7.wav"
		r8 = "data/s8.wav"
		r_sounds = [r1, r2, r3, r4, r5, r6, r7, r8]
		rsnr =random.randint(0,7)
		try:
			rs = pygame.mixer.Sound(r_sounds[rsnr])
		except:
			print ("could not open "+str(rsnr))
		print ("soundnr: "+str(rsnr))
		rs.play()
	if act == 53: #happy or sad
		if sadFace:
			sad.play()
		else:
			happy.play()
	#if act == 73:
		
	#act 21-27 should be game 2
	#act 28-40 should be game 3
	
	info_new=[radius, eyex, eyey, sadFace]
	
	return info_new
	

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
	try:
		changePin = int(changepin) #cast changepin to an int
	except:
		print("changepin is not an int")
	else:
		info = [0, 0, 0]
		sadFace = 0
		robomove(changePin, info, sadFace)
	response = make_response(redirect(url_for('index')))
	return(response)

def data_listener():
	global screen, IP
	
	#Connect the UDP_Port
	try:
		sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
		sock.bind((UDP_IP, UDP_PORT))
		print('***** UDP SOCKET opened ******')
	except:
		print('Could not open UDP SOCKET!')
	
	info = [ 0.8, 0, 0]
	radius= info[0]
	eyex= info[1]
	eyey= info[2]
	sadFace = 0

	DrawFace(eyex, eyey, radius)
	
	try:
		while 1:
			newdata, addr = sock.recvfrom(30) 
			print('***** received data from app ******')
			newdata = newdata.replace("'","")
			newdata = newdata.replace("[","")
			newdata = newdata.replace("]","")
			newdata = int(newdata)
			print (addr)
			IP = addr[0]
			if newdata == 73:
				continue
			print("vor robomove:" + str(sadFace))
			info = robomove(newdata, info, sadFace)
			radius= info[0]
			eyex= info[1]
			eyey= info[2]
			sadFace = info[3]
			if sadFace == 0:
				DrawFace(eyex, eyey, radius)
			elif sadFace == 1:
				print("vor drawsadface")
				DrawSadFace(eyex, eyey, radius)
	finally:
		pygame.quit()

def DrawFace(eyex, eyey, radius):
	global screen
	screen.fill((0, 0, 0))
	pygame.draw.circle(screen, (100,250,250), (163,100), 60) #lefteye
	pygame.draw.circle(screen, (100,250,250), (491,100), 60) #righteye
	pygame.draw.circle(screen, (10,10,10), (163+eyex,100+eyey), 20) #leftpupil
	pygame.draw.circle(screen, (10,10,10), (491+eyex,100+eyey), 20) #rightpupil
	pygame.draw.arc(screen, (100,200,200), (57, -30, 540, 400), 4.7-radius, 4.7+radius, 20) #smile
	pygame.display.flip()
	
def DrawSadFace(eyex, eyey, radius):
	global screen
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
	UDP_Buffer = []
	UDP_Buffer.append(str(i))
	#BufferToSend = list(UDP_Buffer)
	BufferToSend = UDP_Buffer
	UDP_Buffer = []
	try:
		SendToApp.sendto(bytes(BufferToSend), (IP, UDP_PORT+1))
	except socket.error, BufferToSend:
		print 'Error Code: ' + str(BufferToSend[0]) + 'Message ' + BufferToSend[1]
		IP = ''
		#sys.exit()

def BatteryInfo():
	global bat_prozent
	info = [0, 0, 0]
	sadFace = 0
	bat = ""
	count = 0
	while True:
		count += 1
		robomove(0, info, sadFace)
		"""
		ser.write("8")
		bat = str(ser.readline())
		bat = bat.strip(' \t\n\r')
		print("Abfrage " + str(count))
		try:
			bat = int(bat)
			#weiterverarbeitung des wertes
		except:
			print("no data available, recieved message: " + str(bat))
		bat_prozent = ((bat-242)/4.95)
		print(bat)
		print(bat_prozent)
		bat_prozent=int(round(bat_prozent))
		print(bat_prozent)
		sendtogui(bat_prozent)"""
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
				sendtogui(readRFIDnumber)
				readRFIDnumber="empty"	
			time.sleep(0.1)
			
	finally:
		pygame.quit()
		ser.close()

if __name__ == '__main__':
	main()
