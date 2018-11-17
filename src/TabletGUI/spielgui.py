#/usr/bin/env python
#gamegui 3.4.17

import socket
import time
import sys
import pygame, Buttons
import random
import threading
from pygame.locals import *
#write_text(self, surface, text, text_color, length, height, x, y)


#Initialize pygame
pygame.init()

"""GLOBAL"""
#Socketverbindung
UDP_PORT = 9000
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
print('\nUDP socket '+ str(UDP_PORT)+' opened for sending! '  )
UDP_IP=''
#serversocket
try:
	robo = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	robo.bind((UDP_IP, UDP_PORT+1))
	print('***** UDP SOCKET opened ******')
except:
	print('Could not open UDP SOCKET!')

# Comment out/in the line you need:
#in testing mode, just for yourself, use following line, without the fullscreen option:
screen =pygame.display.set_mode((1275,750))
#in testing mode with children, use fullscreen option:
#screen =pygame.display.set_mode((1275,750),pygame.FULLSCREEN)

GREEN = (107,142,35) #buttongreen
WHITE = (255,255,255)
BLACK = (0,0,0)
YELLOW = (255,255,0)
RED = (255,0,0)
DGREEN = (0, 255, 0)
DBLUE = (0,0,128)
B_LENGTH = 200 #Button length
B_HEIGHT = 100 #Button height
S_LENGTH = 50 #Symbol length
S_HEIGHT = 50 #Symbol height
SCREEN_COLOR = (0, 104, 139) #turqois

rfid="empty"
isFilled = False
GameActive = False
bat_prozent = 0

def ipauswahl():
	#ip waehlen
	global UDP_IP
	screen.fill(SCREEN_COLOR)#hellblau (30,144,255)
	pygame.display.flip()
	pygame.display.set_caption("Config")
	text=Buttons.Button()
	text.write_text(screen, "IP Adresse waehlen", BLACK, B_LENGTH, B_HEIGHT, 50, 0)
	ippk=Buttons.Button()
	ipelse=Buttons.Button()
	iphotspot = Buttons.Button()
	ippk.create_button(screen, GREEN, 100, 100, B_LENGTH,    B_HEIGHT,       "ProjectKitchen", WHITE, 0) #dunkelgruen button, weisse schrift
	ipelse.create_button(screen, GREEN, 400, 100, B_LENGTH,    B_HEIGHT,      "ArtLab: 192.168.1.137", WHITE, 0)
	iphotspot.create_button(screen, GREEN, 200, 300, B_LENGTH,    B_HEIGHT,      "HotspotIP: 192.168.0.100", WHITE, 0)
	pygame.display.flip()
	ip = 0
	while ip == 0:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()
				sys.exit()
			elif event.type == pygame.KEYDOWN:
				if event.key == pygame.K_ESCAPE or event.unicode == 'q':
					print "bye!"
					pygame.quit()
					sys.exit()
			elif event.type == MOUSEBUTTONDOWN:
				if ippk.pressed(pygame.mouse.get_pos()):
					UDP_IP = '10.10.10.104'
					ip =1
				if ipelse.pressed(pygame.mouse.get_pos()):
					UDP_IP = '192.168.1.137'
					ip = 1
				if iphotspot.pressed(pygame.mouse.get_pos()):
					UDP_IP = '192.168.0.100'
					ip = 1
	sendtopi("IPcheck")

def gamemenue():
	#Hauptmenue
	global GameActive
	GameActive = False
	#ScreenUpdate()
	sendtopi("face;answer;correct") # guaranteing a happy face
	sendtopi("sound;stop") # stopping any playing sound
	screen.fill(SCREEN_COLOR)#hellblau
	pygame.display.flip()
	pygame.display.set_caption("Hauptmenue")
	text=Buttons.Button()
	text.write_text(screen, "Hauptmenue", BLACK, B_LENGTH, B_HEIGHT, 50, 0)
	spiel1=Buttons.Button()
	spiel1Symbol=Buttons.Button()
	spiel2=Buttons.Button()
	spiel2Symbol=Buttons.Button()
	spiel3=Buttons.Button()
	spiel3Symbol=Buttons.Button()
	spiel4=Buttons.Button()
	spiel4Symbol=Buttons.Button()
	admin =Buttons.Button()
	spiel1.create_button(screen, GREEN, 100, 100, B_LENGTH,    B_HEIGHT,       "Spiel 1", WHITE, 0) #dunkelgruen button, weisse schrift
	spiel1Symbol.create_button(screen, GREEN, 230, 125, S_LENGTH,    S_HEIGHT,       "", WHITE, "data/construction.png")
	spiel2.create_button(screen, GREEN, 100, 300, B_LENGTH,    B_HEIGHT,       "Spiel 2", WHITE, 0)
	spiel1Symbol.create_button(screen, GREEN, 230, 325, S_LENGTH,    S_HEIGHT,       "", WHITE, "data/spiel2.png")
	spiel3.create_button(screen, GREEN, 400, 100, B_LENGTH,    B_HEIGHT,       "Spiel 3", WHITE, 0)
	spiel3Symbol.create_button(screen, GREEN, 530, 125, S_LENGTH,    S_HEIGHT,       "", WHITE, "data/spiel3.png")
	spiel4.create_button(screen, GREEN, 400, 300, B_LENGTH,    B_HEIGHT,       "Spiel 4", WHITE, 0)
	spiel4Symbol.create_button(screen, GREEN, 530, 325, S_LENGTH,    S_HEIGHT,       "", WHITE, "data/spiel4.png")
	admin.create_button(screen, GREEN, 250, 500, B_HEIGHT,    B_HEIGHT,       "Robot Control", WHITE, "data/gear.png")
	pygame.display.flip()

	while True:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()
				sys.exit()
			elif event.type == pygame.KEYDOWN:
				if event.key == pygame.K_ESCAPE or event.unicode == 'q':
					print "bye!"
					pygame.quit()
					sys.exit()
			elif event.type == MOUSEBUTTONDOWN:
				if spiel1.pressed(pygame.mouse.get_pos()):
					game1()
				elif spiel2.pressed(pygame.mouse.get_pos()):
					game2()
				elif spiel3.pressed(pygame.mouse.get_pos()):
					game3()
				elif admin.pressed(pygame.mouse.get_pos()):
					RoboControl()
				elif spiel4.pressed(pygame.mouse.get_pos()):
					game4()

def game1():
	global isFilled, rfid
	global GameActive
	#isFilled = FALSE
	randomsounds = ["data/rot.wav", "data/gruen.wav", "data/blau.wav", "data/gelb.wav", "data/quader.wav", "data/wuerfel.wav", "data/zyl.wav"]
	red = ["15003E321801","1700181F0E1E","15003E2CD9DE"]
	green = ["170019D8A472","16000468BFC5"]
	blue = ["170018DA65B0", "170018323D00"]
	yellow = ["17001832A29F","170018A4B61D"]
	quader = ["170019D8A472","17001832A29F","1700181F0E1E","170018323D00"]
	wuerfel = ["170018DA65B0","15003E2CD9DE","16000468BFC5","170018A4B61D"]
	zylinder = ["15003E321801"]
	#not used yet: "170019A92186"
	screen.fill(SCREEN_COLOR)#hellblau
	pygame.display.flip()
	back=Buttons.Button()
	back.create_button(screen, GREEN, 100, 100, B_HEIGHT, B_HEIGHT, "", WHITE, 'data/home.png')
	text=Buttons.Button()
	text.write_text(screen, "GAME ONE", BLACK, B_LENGTH, B_HEIGHT, 50, 0)
	#sendcommand to robot to play sound "spiel 1 Erklaerung"
	sendtopi("sound;play;data/spiel1.wav")
	start=Buttons.Button()
	start.create_button(screen, GREEN, 400, 200, B_LENGTH, B_HEIGHT, "Start", WHITE, 0)
	starting=0
	pygame.display.flip()
	#Startbildschirm
	while starting==0:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()
				sys.exit()
			elif event.type == pygame.KEYDOWN:
				if event.key == pygame.K_ESCAPE or event.unicode == 'q':
					print "bye!"
					pygame.quit()
					sys.exit()
			elif event.type == MOUSEBUTTONDOWN:
				if back.pressed(pygame.mouse.get_pos()):
					gamemenue()
				elif start.pressed(pygame.mouse.get_pos()):
					starting = 1
	#START SCREEN
	screen.fill(SCREEN_COLOR)#hellblau
	pygame.display.flip()
	back=Buttons.Button()
	back.create_button(screen, GREEN, 100, 100, B_HEIGHT, B_HEIGHT, "", WHITE, 'data/home.png')
	text=Buttons.Button()
	text.write_text(screen, "Spiel 1", BLACK, B_LENGTH, B_HEIGHT, 50, 0)
	text2=Buttons.Button()
	#text2.write_text(screen, "LISTEN TO ROBOT", BLACK, B_LENGTH, B_HEIGHT, 300, 200)
	pygame.display.flip()
	#rfid = ""
	#isFilled= False
	while True:
		GameActive = True
		for event in pygame.event.get():# maybe need to copy this also into the while ans == wrong
			if event.type == pygame.QUIT:
				pygame.quit()
				sys.exit()
			elif event.type == pygame.KEYDOWN:
				if event.key == pygame.K_ESCAPE or event.unicode == 'q':
					print "bye!"
					pygame.quit()
					sys.exit()
			elif event.type == MOUSEBUTTONDOWN:
				if back.pressed(pygame.mouse.get_pos()):
					gamemenue()
		#random sound ran_s from game1 (farbe/form) need to be chosen
		###maybe display the sought object
		#ran_s = random.shuffle(14,20)
		random.shuffle(randomsounds)
		ran_i = random.randint(0,6)
		ran_s = randomsounds[ran_i]
		if ran_s == "data/blau.wav":
			searched = blue
			draw_objects(1)
		elif ran_s == "data/gelb.wav":
			searched = yellow
			draw_objects(2)
		elif ran_s == "data/gruen.wav":
			searched = green
			draw_objects(3)
		elif ran_s == "data/rot.wav":
			searched = red
			draw_objects(4)
		elif ran_s == "data/zyl.wav":
			searched = zylinder
			draw_objects(5)
		elif ran_s == "data/quader.wav":
			searched = quader
			draw_objects(6)
		elif ran_s == "data/wuerfel.wav":
			searched = wuerfel
			draw_objects(7)
		ans=0
		#as long as answer ist incorrect:
		while ans == 0:
			for event in pygame.event.get():# maybe need to copy this also into the while ans == wrong
				if event.type == pygame.QUIT:
					pygame.quit()
					sys.exit()
				elif event.type == pygame.KEYDOWN:
					if event.key == pygame.K_ESCAPE or event.unicode == 'q':
						print "bye!"
						pygame.quit()
						sys.exit()
				elif event.type == MOUSEBUTTONDOWN:
					if back.pressed(pygame.mouse.get_pos()):
						gamemenue()
			#randomly selected sound will be played
			sendtopi("sound;play;"+ran_s)

			#checking if data is right
			#if correct ans = 1
			while isFilled == False: #print("Waiting for data")
				for event in pygame.event.get():# maybe need to copy this also into the while ans == wrong
					if event.type == pygame.QUIT:
						pygame.quit()
						sys.exit()
					elif event.type == pygame.KEYDOWN:
						if event.key == pygame.K_ESCAPE or event.unicode == 'q':
							print "bye!"
							pygame.quit()
							sys.exit()
					elif event.type == MOUSEBUTTONDOWN:
						if back.pressed(pygame.mouse.get_pos()):
							gamemenue()
			print(rfid)
			for r in searched:
				if r == rfid:
					ans = 1
			isFilled = False
			if ans == 0:
				sendtopi("sound;play;data/wrong.wav")# wrong sound
				sendtopi("face;answer;wrong")# sad face
				print("rfid:"+str(rfid))
				text2.write_text(screen, "Das ist leider der falsche Baustein.   Versuche es noch einmal.", BLACK, B_LENGTH, B_HEIGHT, 300, 300)
				pygame.display.flip()
				time.sleep(4) #wait 5 seconds for sound to be over
		sendtopi("sound;play;data/correct.wav")# welldone sound
		sendtopi("face;answer;correct")# happy face
		screen.fill(SCREEN_COLOR)
		text2.write_text(screen, "Gut gemacht! Das war der richtige Baustein.", BLACK, B_LENGTH, B_HEIGHT, 300, 300)
		pygame.display.flip()
		time.sleep(3) #wait 4 seconds for sound to be over

def draw_objects(info):
	screen.fill(SCREEN_COLOR)#hellblau
	pygame.display.flip()
	back=Buttons.Button()
	back.create_button(screen, GREEN, 100, 100, B_HEIGHT, B_HEIGHT, "", WHITE, 'data/home.png')
	text=Buttons.Button()
	text.write_text(screen, "Spiel 1", BLACK, B_LENGTH, B_HEIGHT, 50, 0)
	text2=Buttons.Button()
	if (0<info<5):
		if info == 1:
			color = DBLUE
			text2.write_text(screen, "Suche einen blauen Baustein", color, B_LENGTH, B_HEIGHT, 300, 200)
		if info == 2:
			color = YELLOW
			text2.write_text(screen, "Suche einen gelben Baustein", color, B_LENGTH, B_HEIGHT, 300, 200)
		if info == 3:
			color = DGREEN
			text2.write_text(screen, "Suche einen gruenen Baustein", color, B_LENGTH, B_HEIGHT, 300, 200)
		if info == 4:
			color = RED
			text2.write_text(screen, "Suche einen roten Baustein", BLACK, B_LENGTH, B_HEIGHT, 300, 200)
		#pygame.draw.rect(screen, color, (x,y,width,height), thickness)
		pygame.draw.rect(screen, color, (300,400, B_LENGTH, B_HEIGHT))
		pygame.draw.rect(screen, color, (600,400, B_HEIGHT, B_HEIGHT))
		pygame.draw.circle(screen, color, (850,450),B_HEIGHT/2)
	elif info>4:
		if info == 5:
			text2.write_text(screen, "Suche einen Zylinder", BLACK, B_LENGTH, B_HEIGHT, 300, 200)
			pygame.draw.circle(screen, DBLUE, (200,450),B_HEIGHT/2)
			pygame.draw.circle(screen, YELLOW, (400,450),B_HEIGHT/2)
			pygame.draw.circle(screen, DGREEN, (600,450),B_HEIGHT/2)
			pygame.draw.circle(screen, RED, (800,450),B_HEIGHT/2)
		elif info == 6:
			text2.write_text(screen, "Suche einen Quader", BLACK, B_LENGTH, B_HEIGHT, 300, 200)
			pygame.draw.rect(screen, DBLUE, (50,400, B_LENGTH, B_HEIGHT))
			pygame.draw.rect(screen, YELLOW, (350,400, B_LENGTH, B_HEIGHT))
			pygame.draw.rect(screen, DGREEN, (650,400, B_LENGTH, B_HEIGHT))
			pygame.draw.rect(screen, RED, (950,400, B_LENGTH, B_HEIGHT))
		elif info == 7:
			text2.write_text(screen, "Suche einen Wuerfel", BLACK, B_LENGTH, B_HEIGHT, 300, 200)
			pygame.draw.rect(screen, DBLUE, (200,400, B_HEIGHT, B_HEIGHT))
			pygame.draw.rect(screen, YELLOW, (400,400, B_HEIGHT, B_HEIGHT))
			pygame.draw.rect(screen, DGREEN, (600,400, B_HEIGHT, B_HEIGHT))
			pygame.draw.rect(screen, RED, (800,400, B_HEIGHT, B_HEIGHT))
	pygame.display.flip()

def game2():
	img_size= 300
	screen.fill(SCREEN_COLOR)#hellblau
	pygame.display.flip()
	back=Buttons.Button()
	back.create_button(screen, GREEN, 100, 100, B_HEIGHT, B_HEIGHT, "", WHITE, 'data/home.png')
	#load font, prepare values
	text=Buttons.Button()
	text.write_text(screen, "Spiel 2", BLACK, B_LENGTH, B_HEIGHT, 50, 0)

	#GAME 2 screen
	sendtopi("sound;play;data/robogame2.wav")# Spiel2 Erklaerung
	start=Buttons.Button()
	start.create_button(screen, GREEN, 400, 200, B_LENGTH, B_HEIGHT, "Start", WHITE, 0)
	starting=0
	pygame.display.flip()
	#Startbildschirm
	while starting==0:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()
				sys.exit()
			elif event.type == pygame.KEYDOWN:
				if event.key == pygame.K_ESCAPE or event.unicode == 'q':
					print "bye!"
					pygame.quit()
					sys.exit()
			elif event.type == MOUSEBUTTONDOWN:
				if back.pressed(pygame.mouse.get_pos()):
					gamemenue()
				elif start.pressed(pygame.mouse.get_pos()):
					starting = 1

	positions = [(150,375),(500,375),(850,375)]

	#Buttons
	happy=Buttons.Button()
	sad=Buttons.Button()
	angry=Buttons.Button()
	emoticons = [happy, sad, angry]
	i = 0
	story = 0
	while story == 0:
		for event in pygame.event.get():
				if event.type == pygame.QUIT:
					pygame.quit()
					sys.exit()
				elif event.type == pygame.KEYDOWN:
					if event.key == pygame.K_ESCAPE or event.unicode == 'q':
						print "bye!"
						pygame.quit()
						sys.exit()
				elif event.type == MOUSEBUTTONDOWN:
					if back.pressed(pygame.mouse.get_pos()):
						gamemenue()
		random.shuffle(positions)
		screen.fill(SCREEN_COLOR)#hellblau
		pygame.display.flip()
		back=Buttons.Button()
		back.create_button(screen, GREEN, 100, 100, B_HEIGHT, B_HEIGHT, "", WHITE, 'data/home.png')
		text=Buttons.Button()
		text.write_text(screen, "Spiel 2", BLACK, B_LENGTH, B_HEIGHT, 50, 0)
		text2=Buttons.Button()
		i = i+1 # FIRST ROUND
		#displaying of emoticons !
		if (i % 2) == 0: #2.; 4.; 6.; Runde: bub
			happy.create_button(screen, WHITE, positions[0][0], positions[0][1], img_size, img_size, "", WHITE, 'data/happyB.jpg')
			sad.create_button(screen, WHITE, positions[1][0], positions[1][1], img_size, img_size, "", WHITE, 'data/sadB.jpg')
			angry.create_button(screen, WHITE, positions[2][0], positions[2][1], img_size, img_size, "", WHITE, 'data/angryB.jpg')
		else: #1.; 3.; 5.; Runde: maedchen
			happy.create_button(screen, WHITE, positions[0][0], positions[0][1], img_size, img_size, "", WHITE, 'data/happyM.png')
			sad.create_button(screen, WHITE, positions[1][0], positions[1][1], img_size, img_size, "", WHITE, 'data/sadM.png')
			angry.create_button(screen, WHITE, positions[2][0], positions[2][1], img_size, img_size, "", WHITE, 'data/angryM.png')
		pygame.display.flip()
		#playing sound, depending on which round this is
		if i == 1:
			storysound = "data/2robory1.wav"
			emo = sad
		elif i == 2:
			storysound = "data/2robory2.wav"
			emo = angry
		elif i == 3:
			storysound = "data/2robory3.wav"
			emo = angry
		elif i == 4:
			storysound = "data/2robory4.wav"
			emo = sad
		elif i == 5:
			storysound = "data/2robory5.wav"
			emo = happy
		elif i == 6:
			storysound = "data/2robory6.wav"
			emo = happy
			story = 1 #last sound. after this, end while loop
		sendtopi("sound;play;"+storysound) #SOund 1_spiel 2 blabla *ist happy. lets see if u find happy face, just click on it*
		ans = 0
		while ans == 0:
			for event in [pygame.event.wait()]+pygame.event.get():# maybe need to copy this also into the while ans == wrong
				if event.type == pygame.QUIT:
					pygame.quit()
					sys.exit()
				elif event.type == pygame.KEYDOWN:
					if event.key == pygame.K_ESCAPE or event.unicode == 'q':
						print "bye!"
						pygame.quit()
						sys.exit()
				elif event.type == MOUSEBUTTONDOWN:
					if back.pressed(pygame.mouse.get_pos()):
						gamemenue()
					#checking if data is right
					#if correct ans = 1
					for e in emoticons:
						if e.pressed(pygame.mouse.get_pos()):
							if e == emo:
								print ("right")
								sendtopi("sound;play;data/correct.wav")# welldone sound
								sendtopi("face;answer;correct")# happy face
								time.sleep(3)#bis sound fertig
								ans =1
							else:
								print("wrong")
								sendtopi("sound;play;data/wrong.wav")# wrong sound
								sendtopi("face;answer;wrong")# sad face
								time.sleep(4)#bis sound fertig
								sendtopi("sound;play;"+storysound)#"sound"
	screen.fill(SCREEN_COLOR)#hellblau
	pygame.display.flip()
	back=Buttons.Button()
	back.create_button(screen, GREEN, 100, 100, B_HEIGHT, B_HEIGHT, "", WHITE, 'data/home.png')
	start=Buttons.Button()
	start.create_button(screen, GREEN, 400, 200, B_LENGTH, B_HEIGHT, "Neustart", WHITE, 0)
	pygame.display.flip()
	#Startbildschirm
	while True:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()
				sys.exit()
			elif event.type == pygame.KEYDOWN:
				if event.key == pygame.K_ESCAPE or event.unicode == 'q':
					print "bye!"
					pygame.quit()
					sys.exit()
			elif event.type == MOUSEBUTTONDOWN:
				if back.pressed(pygame.mouse.get_pos()):
					gamemenue()
				if start.pressed(pygame.mouse.get_pos()):
					game2()

def game3():
	screen.fill(SCREEN_COLOR)#hellblau
	pygame.display.flip()
	back=Buttons.Button()
	randomsounds = ["data/sheep.wav", "data/bird.wav", "data/cow.wav", "data/dogs.wav", "data/cat.wav", "data/elephant.wav"]
	back.create_button(screen, GREEN, 100, 100, B_HEIGHT, B_HEIGHT, "", WHITE, 'data/home.png')
	#load font, prepare values
	text=Buttons.Button()
	text.write_text(screen, "Spiel 3", BLACK, B_LENGTH, B_HEIGHT, 50, 0)
	#pygame.display.flip()
	#sendcommand to robot to play sound "spiel 3 Erklaerung"
	sendtopi("sound;play;data/robogame3.wav")
	start=Buttons.Button()
	start.create_button(screen, GREEN, 400, 200, B_LENGTH, B_HEIGHT, "Start", WHITE, 0)
	starting=0
	pygame.display.flip()
	#Startbildschirm
	while starting==0:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()
				sys.exit()
			elif event.type == pygame.KEYDOWN:
				if event.key == pygame.K_ESCAPE or event.unicode == 'q':
					print "bye!"
					pygame.quit()
					sys.exit()
			elif event.type == MOUSEBUTTONDOWN:
				if back.pressed(pygame.mouse.get_pos()):
					gamemenue()
				elif start.pressed(pygame.mouse.get_pos()):
					starting = 1
	# screen.fill((30,144,255))#hellblau
	# pygame.display.flip()
	# back=Buttons.Button()
	# back.create_button(screen, GREEN, 100, 100, B_LENGTH, B_HEIGHT, "ZURUECK", WHITE, 0)
	# text=Buttons.Button()
	# text.write_text(screen, "GAME THREE", BLACK, B_LENGTH, B_HEIGHT, 50, 0)
	while True:
		for event in pygame.event.get():# maybe need to copy this also into the while ans == wrong
			if event.type == pygame.QUIT:
				pygame.quit()
				sys.exit()
			elif event.type == pygame.KEYDOWN:
				if event.key == pygame.K_ESCAPE or event.unicode == 'q':
					print "bye!"
					pygame.quit()
					sys.exit()
			elif event.type == MOUSEBUTTONDOWN:
				if back.pressed(pygame.mouse.get_pos()):
					gamemenue()
		#random sound ran_s from game3 (animal) need to be chosen
		#displaying animals, am besten random positions
		random.shuffle(randomsounds)
		ran_i = random.randint(0,5)
		ran_s = randomsounds[ran_i]
		#ran_s = random.randint(29,34)
		#right_sound = ran_s+6
		screen.fill(SCREEN_COLOR)#hellblau
		pygame.display.flip()
		back.create_button(screen, GREEN, 100, 100, B_HEIGHT, B_HEIGHT, "", WHITE, 'data/home.png')
		positions = [(150,250),(500,250),(850,250),(150,500),(500,500),(850,500)]
		random.shuffle(positions)
		cow=Buttons.Button()
		cow.create_button(screen, GREEN, positions[0][0], positions[0][1], 300, 200, "", WHITE, 'data/cow.jpg')
		dog=Buttons.Button()
		dog.create_button(screen, GREEN, positions[1][0], positions[1][1], 300, 200, "", WHITE, 'data/dogs.jpg')
		cat=Buttons.Button()
		cat.create_button(screen, GREEN, positions[2][0], positions[2][1], 300, 200, "", WHITE, 'data/cat.jpg')
		elephant=Buttons.Button()
		elephant.create_button(screen, GREEN, positions[3][0], positions[3][1], 300, 200, "", WHITE, 'data/elephant.jpg')
		sheep=Buttons.Button()
		sheep.create_button(screen, GREEN, positions[4][0], positions[4][1], 300, 200, "", WHITE, 'data/sheepherd.jpg')
		bird=Buttons.Button()
		bird.create_button(screen, GREEN, positions[5][0], positions[5][1], 300, 200, "", WHITE, 'data/bird.jpg')
		pygame.display.flip()
		animals = [cow, dog, cat, elephant, sheep, bird]
		if ran_s == "data/bird.wav":#bird
			searched = bird
			right_sound = "data/robobird.wav"
		elif ran_s == "data/cat.wav":#cat
			searched = cat
			right_sound = "data/robocatser.wav"
		elif ran_s == "data/cow.wav":#cow
			searched = cow
			right_sound = "data/robocow.wav"
		elif ran_s == "data/dogs.wav":#dog
			searched = dog
			right_sound = "data/robodogs.wav"
		elif ran_s == "data/elephant.wav":#elephant
			searched = elephant
			right_sound = "data/robofant.wav"
		elif ran_s == "data/sheep.wav":#sheep
			searched = sheep
			right_sound = "data/robeep.wav"
		ans=0
		print(ran_s)
		#randomly selected sound will be played
		sendtopi("sound;play;"+ran_s)#"animal sound"
		#as long as answer ist incorrect:
		while ans == 0:
			for event in [pygame.event.wait()]+pygame.event.get():# maybe need to copy this also into the while ans == wrong
				if event.type == pygame.QUIT:
					pygame.quit()
					sys.exit()
				elif event.type == pygame.KEYDOWN:
					if event.key == pygame.K_ESCAPE or event.unicode == 'q':
						print "bye!"
						pygame.quit()
						sys.exit()
				elif event.type == MOUSEBUTTONDOWN:
					if back.pressed(pygame.mouse.get_pos()):
						gamemenue()
					#checking if data is right
					#if correct ans = 1
					for c in animals:
						if c.pressed(pygame.mouse.get_pos()):
							if c == searched:
								print ("right")
								sendtopi("sound;play;"+right_sound)
								sendtopi("face;answer;correct")# happy face
								time.sleep(3)#bis sound fertig
								ans =1
							else:
								print("wrong")
								sendtopi("sound;play;data/wrong.wav")# wrong sound
								sendtopi("face;answer;wrong")# sad face
								time.sleep(4)#bis sound fertig
								sendtopi("sound;play;"+ran_s)#"animal sound"

def game4():
	global isFilled, rfid
	global GameActive
	bird ="4500B8D1FBD7"
	cat = "4500B8C9C7F3"
	cow = "4500B8FB3A3C"
	dog = "4500B8EC5F4E"
	elephant = "4500B8F3CBC5"
	sheep = "4500B8E04A57"
	screen.fill(SCREEN_COLOR)#hellblau
	pygame.display.flip()
	back=Buttons.Button()
	back.create_button(screen, GREEN, 100, 100, B_HEIGHT, B_HEIGHT, "", WHITE, 'data/home.png')
	text=Buttons.Button()
	text.write_text(screen, "Spiel 4", BLACK, B_LENGTH, B_HEIGHT, 50, 0)
	#sendcommand to robot to play sound "spiel 4 Erklaerung"
	sendtopi("sound;play;data/spiel4.wav")
	start=Buttons.Button()
	start.create_button(screen, GREEN, 400, 200, B_LENGTH, B_HEIGHT, "Start", WHITE, 0)
	starting=0
	pygame.display.flip()
	#Startbildschirm
	while starting==0:
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()
				sys.exit()
			elif event.type == pygame.KEYDOWN:
				if event.key == pygame.K_ESCAPE or event.unicode == 'q':
					print "bye!"
					pygame.quit()
					sys.exit()
			elif event.type == MOUSEBUTTONDOWN:
				if back.pressed(pygame.mouse.get_pos()):
					gamemenue()
				elif start.pressed(pygame.mouse.get_pos()):
					starting = 1
	#START SCREEN
	screen.fill(SCREEN_COLOR)#hellblau
	pygame.display.flip()
	back=Buttons.Button()
	back.create_button(screen, GREEN, 100, 100, B_HEIGHT, B_HEIGHT, "", WHITE, 'data/home.png')
	text=Buttons.Button()
	text.write_text(screen, "Spiel 4", BLACK, B_LENGTH, B_HEIGHT, 50, 0)
	text2=Buttons.Button()
	text2.write_text(screen, "Halte eine Karte mit einem beliebigen Tier an den Roboter", BLACK, B_LENGTH, B_HEIGHT, 300, 200)
	pygame.display.flip()
	GameActive = True
	while True:
		for event in pygame.event.get():# maybe need to copy this also into the while ans == wrong
			if event.type == pygame.QUIT:
				pygame.quit()
				sys.exit()
			elif event.type == pygame.KEYDOWN:
				if event.key == pygame.K_ESCAPE or event.unicode == 'q':
					print "bye!"
					pygame.quit()
					sys.exit()
			elif event.type == MOUSEBUTTONDOWN:
				if back.pressed(pygame.mouse.get_pos()):
					gamemenue()
		#as long as invalid rfid data (baustein statt tier):
		while isFilled == False: #print("Waiting for data")
			for event in pygame.event.get():# maybe need to copy this also into the while ans == wrong
				if event.type == pygame.QUIT:
					pygame.quit()
					sys.exit()
				elif event.type == pygame.KEYDOWN:
					if event.key == pygame.K_ESCAPE or event.unicode == 'q':
						print "bye!"
						pygame.quit()
						sys.exit()
				elif event.type == MOUSEBUTTONDOWN:
					if back.pressed(pygame.mouse.get_pos()):
						gamemenue()
		print(rfid)
		if rfid == bird: # BIRD
			sendtopi("sound;play;data/bird.wav") #animalsound
		elif rfid == cat: # CAT
			sendtopi("sound;play;data/cat.wav") #animalsound
		elif rfid == cow : #COW
			sendtopi("sound;play;data/cow.wav") #animalsound
		elif rfid == dog : #DOG
			sendtopi("sound;play;data/dogs.wav") #animalsound
		elif rfid == elephant: #ELEPHANT
			sendtopi("sound;play;data/elephant.wav") #animalsound
		elif rfid == sheep: #SHEEP
			sendtopi("sound;play;data/sheep.wav") #animalsound
		else:
			sendtopi("sound;play;data/keinTier.wav") #wrong sound
			print("rfid:"+str(rfid))
			time.sleep(4) #wait 5 seconds for sound to be over
		isFilled = False

def RoboControl():
	arrow_size = 150
	arrow_start = 150
	face_size = 2*arrow_size/3
	screen.fill(SCREEN_COLOR)#hellblau
	pygame.display.flip()
	back=Buttons.Button()
	back.create_button(screen, GREEN, 100, 100, B_HEIGHT, B_HEIGHT, "", WHITE, 'data/home.png')
	forward =Buttons.Button()
	forward.create_button(screen, GREEN, arrow_start+125, 275, arrow_size, arrow_size, "", WHITE, 'data/arrow_up.png')
	left =Buttons.Button()
	left.create_button(screen, GREEN, arrow_start, 400, arrow_size, arrow_size, "", WHITE, 'data/arrow_left.png')
	right=Buttons.Button()
	right.create_button(screen, GREEN, arrow_start+250, 400, arrow_size, arrow_size, "", WHITE, 'data/arrow_right.png')
	backward=Buttons.Button()
	backward.create_button(screen, GREEN, arrow_start+125, 525, arrow_size, arrow_size, "", WHITE, 'data/arrow_down.png')
	smileUp =Buttons.Button()
	smileUp.create_button(screen, GREEN, 750, 250, face_size, face_size, "", WHITE, 'data/smile_add.png')
	smileDown =Buttons.Button()
	smileDown.create_button(screen, GREEN, 950, 250, face_size, face_size, "", WHITE, 'data/smile_delete.png')
	eyeUp =Buttons.Button()
	eyeUp.create_button(screen, GREEN, 800, 400, B_LENGTH, B_HEIGHT, "", WHITE, 'data/eye_up.png')
	eyeDown =Buttons.Button()
	eyeDown.create_button(screen, GREEN, 800, 525, B_LENGTH, B_HEIGHT, "", WHITE, 'data/eye_down.png')
	eyeLeft =Buttons.Button()
	eyeLeft.create_button(screen, GREEN, 575, 525, B_LENGTH, B_HEIGHT, "", WHITE, 'data/eye_left.png')
	eyeRight =Buttons.Button()
	eyeRight.create_button(screen, GREEN, 1025, 525, B_LENGTH, B_HEIGHT, "", WHITE, 'data/eye_right.png')
	pygame.display.flip()
	co= 0
	# drive= F, R, B, L
	drive = [0, 0, 0, 0]


	while True:
		#key mode abfrage
		keys=pygame.key.get_pressed()
		if keys[K_LEFT]:
			if keys[K_DOWN]:
				print(str(co)+"LINKSHINTEN!")
				if drive == [0, 0, 1, 1]:
					pass
				else:
					drive = [0, 0, 1, 1]
					sendtopi("move;backwad_left")
			elif keys[K_UP]:
				print(str(co)+"LINKSVORNE!")
				if drive == [1, 0, 0, 1]:
					pass
				else:
					drive = [1, 0, 0, 1]
					sendtopi("move;forward_left")
			else:
				print(str(co)+"LINKS!")
				if drive == [0, 0, 0, 1]:
					pass
				else:
					drive = [0, 0, 0, 1]
					sendtopi("move;left")
			co = co+1
		elif keys[K_RIGHT]:
			if keys[K_DOWN]:
				print(str(co)+"RECHTSHINTEN!")
				if drive == [0, 1, 1, 0]:
					pass
				else:
					drive = [0, 1, 1, 0]
					sendtopi("move;backward_right")
			elif keys[K_UP]:
				print(str(co)+"RECHTSVORNE!")
				if drive == [1, 1, 0, 0]:
					pass
				else:
					drive = [1, 1, 0, 0]
					sendtopi("move;forward_right")
			else:
				print(str(co)+"RECHTS!")
				if drive == [0, 1, 0, 0]:
					pass
				else:
					drive = [0, 1, 0, 0]
					sendtopi("move;right")
			co = co+1
		elif keys[K_UP]:
			print(str(co)+"VORWAERTS!")
			if drive == [1, 0, 0, 0]:
				pass
			else:
				drive = [1, 0, 0, 0]
				sendtopi("move;forward")
			co = co+1
		elif keys[K_DOWN]:
			print(str(co)+"RUECKWAERTS!")
			if drive == [0, 0, 1, 0]:
				pass
			else:
				drive = [0, 0, 1, 0]
				sendtopi("move;backward")
			co = co+1
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit()
				sys.exit()
			elif event.type == pygame.KEYUP:
				print("STOP!")
				if drive == [0, 0, 0, 0]:
					pass
				else:
					sendtopi("move;stop")
					drive = [0, 0, 0, 0]
					print("Sent to pi")
			elif event.type == pygame.KEYDOWN:
				if event.key == pygame.K_ESCAPE or event.unicode == 'q':
					print "bye!"
					pygame.quit()
					sys.exit()
				if event.key == pygame.K_1: #rot
					print "random sound"
					sendtopi("sound;play;data/fabibox/;random")
				if event.key == pygame.K_2: #blau
					print "sound wie gehts"
					sendtopi("sound;play;data/;mood")
				if event.key == pygame.K_3: #weiss
					print "Smile -"
					sendtopi("face;smile;decrease")
				if event.key == pygame.K_4: #gruen
					print "Smile +"
					sendtopi("face;smile;increase")
			elif event.type == MOUSEBUTTONDOWN:
				if back.pressed(pygame.mouse.get_pos()):
					gamemenue()
				if forward.pressed(pygame.mouse.get_pos()):
					print "forward!"
					sendtopi("move;forward;step")
				if left.pressed(pygame.mouse.get_pos()):
					print "left!"
					sendtopi("move;left;step")
				if right.pressed(pygame.mouse.get_pos()):
					print "right!"
					sendtopi("move;right;step")
				if backward.pressed(pygame.mouse.get_pos()):
					print "back!"
					sendtopi("move;backward;step")
				if smileUp.pressed(pygame.mouse.get_pos()):
					print "Smile +"
					sendtopi("face;smile;increase")
				if smileDown.pressed(pygame.mouse.get_pos()):
					print "Smile -"
					sendtopi("face;smile;decrease")
				if eyeUp.pressed(pygame.mouse.get_pos()):
					print "eye /\\"
					sendtopi("face;eyes;up")
				if eyeDown.pressed(pygame.mouse.get_pos()):
					print "eye \/"
					sendtopi("face;eyes;down")
				if eyeLeft.pressed(pygame.mouse.get_pos()):
					print "eye <"
					sendtopi("face;eyes;left")
				if eyeRight.pressed(pygame.mouse.get_pos()):
					print "eye >"
					sendtopi("face;eyes;right")

def sendtopi(i):
	global UDP_IP, UDP_PORT
	BytesToSend = bytes(":RUN:" + str(i) + ":EOL:")
	try:
		sock.sendto(BytesToSend, (UDP_IP, UDP_PORT))
		print(BytesToSend)
	except socket.error, msg:
		print 'Error Code: ' + str(msg[0]) + 'Message ' + msg[1]
		sys.exit()

def ReadFromPi():
	global rfid, isFilled, GameActive, bat_prozent
	count = 0
	startFlag = ":RUN:"
	endFlag = ":EOL:"
	#rfid = ""
	while 1:
		count += 1
		while isFilled != False:
			time.sleep(0.1)
		data = ""
		rfid = ""
		tempData, addr = robo.recvfrom(30)
		print("Abfrage " + str(count))

		print(data)
		"""data = data.replace("'","")
		data = data.replace("[","")
		data_clean = data.replace("]","")"""
		data += tempData
		startFlagIndex = data.find(startFlag)
		if startFlagIndex != -1:
			data = data[startFlagIndex + len(startFlag):]

		endFlagIndex = data.find(endFlag)
		if endFlagIndex == -1:
			continue

		data = data[:endFlagIndex]
		dataArray = data.split(';')
		if dataArray[0] == "battery":
			try:
				bat_prozent = int(dataArray[1])
			except:
				print("Battery Status couldn't be checked")
		elif dataArray[0] == "rfid":
			rfid = dataArray[1]
		"""
		try:
			bat_prozent = int(data_clean)
		except:
			rfid = data_clean
		finally:"""
		print(dataArray)
		print(bat_prozent)
		print(rfid)
		if GameActive and (rfid != ""):
			isFilled = True

def ScreenUpdate():
	global bat_prozent

	while True:
		akku = "Akku: "+str(bat_prozent)+"V"
		print("akku aktualisierung")
		pygame.draw.rect(screen, SCREEN_COLOR, (950,100, B_LENGTH, B_HEIGHT))
		#paint.create_button(screen, , 950, 100, B_LENGTH, B_HEIGHT, "Akku:", BLACK)
		text=Buttons.Button()
		#print("writing text")
		text.write_text(screen, akku, BLACK, B_LENGTH, B_HEIGHT, 950, 100)
		pygame.display.flip()
		#bat_prozent = bat_prozent+1
		time.sleep(3)

def main():
	ReadFromRobo = threading.Thread(target=ReadFromPi)
	ReadFromRobo.daemon = True
	ReadFromRobo.start()
	GameActive = False
	ipauswahl()
	sendtopi("battery;status") #ask for first batteryvoltage
	time.sleep(0.04) #time needed to recieve batteryvoltage
	BatToScreen = threading.Thread(target=ScreenUpdate)
	BatToScreen.daemon = True
	BatToScreen.start()
	gamemenue()



if __name__ == '__main__':
	main()
