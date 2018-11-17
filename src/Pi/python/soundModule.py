import pygame
import random

#own modules
import faceModule
import utils

#globals
lastPlayFile = None

#init
print("initializing soundModule...")
pygame.init()
if pygame.mixer and not pygame.mixer.get_init():
    print ('Warning, no sound')
    pygame.mixer = None

# methods
def close():
    pygame.quit()

def stop():
    pygame.mixer.stop()

def playSoundFile(path):
    sound = None
    try:
        print ("playing sound " + path)
        sound = pygame.mixer.Sound(path)
    except:
        print ("could not open " + path)
    else:
        sound.play()

def playRandom():
    global lastPlayFile
    path = 'data/random/'
    filenames = utils.getFilenames(path)
    playFile = path + random.choice(filenames)
    lastPlayFile = playFile
    playSoundFile(playFile)

def playLastRandom():
    global lastPlayFile
    if not lastPlayFile:
        playRandom()
    playSoundFile(lastPlayFile)

def getRandomSounds():
    return utils.getFilenames('data/random/')

def playMood():
    playsound(["data/","mood"])

# deprecated: use single methods like playRandom()
# this function is called by chooseAction if the robot has to speak/make a sound
def playsound(dataArray):
    stop()
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
            playRandom()
        elif info == "mood":
            print ("mood sound")
            mood_sounds = ["sad","happy"]
            if faceModule.isSad():
                soundpath = soundname+mood_sounds[0]+".wav"
            else:
                soundpath = soundname+mood_sounds[1]+".wav"
    else:
        print ("playing soundfile " + soundname)
        soundpath = soundname
    playSoundFile(soundpath)
